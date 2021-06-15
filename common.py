#!/usr/bin/env python3

from enum import Enum
import collections
import sys, traceback

import time

from threading import Thread, Lock, Condition, RLock
import threading

import traceback

### Utility functions
class bcolors:
  # https://godoc.org/github.com/whitedevops/colors
  DEFAULT = "\033[39m"
  BLACK = "\033[30m"
  RED = "\033[31m"
  GREEN = "\033[32m"
  YELLOW = "\033[33m"
  BLUE = "\033[34m"
  MAGENTA = "\033[35m"
  CYAN = "\033[36m"
  LGRAY = "\033[37m"
  DARKGRAY = "\033[90m"
  FAIL = "\033[91m"
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  OKBLUE = '\033[94m'
  HEADER = '\033[95m'
  LIGHTCYAN = '\033[96m'
  WHITE = "\033[97m"

  ENDC = '\033[0m'
  BOLD = '\033[1m'
  DIM = "\033[2m"
  UNDERLINE = '\033[4m'
  BLINK = "\033[5m"
  REVERSE = "\033[7m"
  HIDDEN = "\033[8m"

  BG_DEFAULT = "\033[49m"
  BG_BLACK = "\033[40m"
  BG_RED = "\033[41m"
  BG_GREEN = "\033[42m"
  BG_YELLOW = "\033[43m"
  BG_BLUE = "\033[44m"
  BG_MAGENTA = "\033[45m"
  BG_CYAN = "\033[46m"
  BG_GRAY = "\033[47m"
  BG_DKGRAY = "\033[100m"
  BG_LRED = "\033[101m"
  BG_LGREEN = "\033[102m"
  BG_LYELLOW = "\033[103m"
  BG_LBLUE = "\033[104m"
  BG_LMAGENTA = "\033[105m"
  BG_LCYAN = "\033[106m"
  BG_WHITE = "\033[107m"

def nonempty_queue_exists(blackboard, admissible_nonempty_keys,
  verbose = False):
  # print("######## nonempty_queue_exists")
  for k in blackboard.keys():
    if k[-6:] == "_queue":
      mutex_k = k[:-6] + "_mutex"
      blackboard[mutex_k].acquire()
      queue_size = len(blackboard[k])
      blackboard[mutex_k].release()
      if verbose:
        print("%s queue_size %d" % (k, queue_size))
        print(blackboard[k])
      if queue_size > 0:
        if verbose:
          print("nonempty k: ", k, queue_size)
        if k not in admissible_nonempty_keys:
          return True
  # print("######## nonempty_queue_exists done")
  return False

# 2019-01-09 some decorators copied from utils cannot import
# because of circular dependency? TODO(j) cleanup
def wrap_instance_method(instance, method_name, wrapper_with_args):
  wrapped_method = wrapper_with_args(getattr(instance, method_name))
  setattr(instance, method_name, wrapped_method)

# 2018-01-11 often we spawn child threads but if they fail at any point
# the parent thread cannot know beyond a callback indirectly that the
# child succeeded, or failed cannot know at all
# this decorator is a tool against that and the notify_func_done, notify_func_fail
# is a way to more directly know that the func finished or failed explicitly
def watchdog_try_to_run_and_notify_failure(result_key,
  notify_func_done = None,
  notify_func_fail = None):
  def decorator(func):
    def wrapper(*args, **kwargs):
      try:
        func(*args, **kwargs)
        if notify_func_done is not None:
          notify_func_done(result_key, "done")
        # notify_func not only catches failures but more directly good outcomes
      except: # 2019-01-11 more generic Errors != Exceptions etc.
        print("failed trying something")
        if notify_func_fail is not None:
          notify_func_fail(result_key, sys.exc_info())
        # more directly catch failures
    return wrapper
  return decorator

def call_when_switch_turned_on(obj, switch, switch_mutex):
  def decorator(func):  # func should return void
    def wrapper(*args, **kwargs):
      mutex_obj = getattr(obj, switch_mutex)
      mutex_obj.acquire()  # optionally, non-blocking acquire
      switch_state = getattr(obj, switch)
      if (not switch_state):
        # tells you lock state @ this call, it may be released immediately after
        mutex_obj.release() # 2019-01-09 SUPER #IMPORTANT
        print("call_when_switch_turned_on: off, noop")
        raise Exception("call_when_switch_turned_on: off, noop")
      res = func(*args, **kwargs)
      mutex_obj.release()
      return res
    return wrapper
  return decorator

def wrap_with_prints(pre_msg, post_msg):
  '''useful for for example printing with color'''
  def decorator(func):
    def wrapper(*args, **kwargs):
      print(pre_msg, end = '')
      res = func(*args, **kwargs)
      print(post_msg, end='')
      return res
    return wrapper
  return decorator

def isfloat(x):
  try:
    a = float(x)
  except ValueError:
    return False
  else:
    return True

def isint(x):
  try:
    a = float(x)
    b = int(a)
  except ValueError:
    return False
  else:
    return a == b

### EVENT INFRA

# 2019-01-07 event design pattern
# tool to deal with temporal architecture OF EXECUTION
class EventThread(Thread):
  """Thread class with a stop() method. The thread itself has to check
  regularly for the stopped() condition."""

  def __init__(self,
    callback = None,
    oneshot = True,
    delay_secs = None,
    *args, **kwargs):
    super(EventThread, self).__init__(*args, **kwargs)
    self.callback = callback
    self.oneshot = oneshot
    self.delay_secs = delay_secs

    # IMPORTANT, threading library
    # wants to call a function _stop()
    # so we must name this to not override that
    self._stop_event = threading.Event()

  def terminate(self):
    self._stop_event.set()

  def stopped(self):
    return self._stop_event.isSet()

  def run(self):
    # print "starting up stoppable thread"
    if self.delay_secs is not None:
      time.sleep(self.delay_secs)
    super(EventThread, self).run()
    if self.oneshot:
      self.terminate()
    if self.callback:
      self.callback()

class Event(object):
  def __init__(self, event_id, *args, **kwargs):
    self.event_id = event_id

    # note: on construction, does NOT have/need a blackboard
    # when dispatched, it MAY have a blackboard (access to actors)
    self.blackboard = None

    # note: on construction, does NOT have/need an ED
    # when dispatched, it MUST have an ED (access to dispatch, events)
    self.event_dispatch = None
    # not fixed, can be changed across different dispatches

    # 2019-01-26
    # give events a pointer
    # to the prior invoking event
    # optional for events to use
    # to be polymorphic w.r.t.
    # (one of the big motivations)
    self.dispatching_event = None

  # methods for the child to override
  def get_id(self):
    return self.__class__.__name__ + "@" + str(self.event_id)

  def dispatch(self, event_dispatch, *args, **kwargs):
    # CAN EITHER PASS IN ARGS OR KWARGS HERE
    # OR SET THEM IN CONSTRUCTOR
    # OR SET THEM IN THE BLACKBOARD
    # unlike deserialize / finish, happens in its own thread
    if self.get_id() not in event_dispatch.mutex_registry:
      # when dispatched, it MUST have an ED (access to dispatch, events)
      self.event_dispatch = event_dispatch

      # dynamic registration in ED mutex_registry
      # will let OTHER events block on this event's state
      # ex:
      # event_dispatch.mutex_registry[self.get_id()] = Lock()

      # when dispatched, it MAY have a blackboard (access to actors)
      # ex:
      # self.blackboard = args[0]

  def finish(self, event_dispatch, *args, **kwargs):
    # unlike deserialize / dispatch, involves other events
    raise NotImplementedError

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    # up to the Event class to define
    # returns 2 tuples, constructor_args, dispatch_args
    # unlike dispatch / finish, involves no instance
    raise NotImplementedError

class TryFailEvent(Event):
  def __init__(self, event_id, *args, **kwargs):
    super(TryFailEvent, self).__init__(
      event_id, *args, **kwargs)

    wrap_instance_method(self,
      "dispatch",
      watchdog_try_to_run_and_notify_failure(
        "dispatch",
        self.dispatch_done,
        self.catch_exception))

    wrap_instance_method(self,
      "finish",
      watchdog_try_to_run_and_notify_failure(
        "finish",
        self.finish_done,
        self.catch_exception))

    # some cases you want to noop finish completely
    # in other cases you want to turn it off temporarily but keep the logic
    # like when an event is interrupted, you want to turn it off
    # then turn it back on when you resume
    self.finish_switch_mutex = Lock()
    self.finish_switch = True
    wrap_instance_method(self, 'finish',
      call_when_switch_turned_on(
        self, "finish_switch",
        "finish_switch_mutex")) # [example] decorator

  # for children classes to override
  # for example, releasing the event mutex_registry lock
  # if you want to handover event's managing of its lock to parent
  def dispatch_done(self, result_key, result):
    pass

  def finish_done(self, result_key, result):
    pass

  def catch_exception(self, result_key, result):
    print("warning: " + self.get_id() + " catch_exception not overriden")
    print("result:", result_key, result)
    try:
      traceback.print_exception(*result)
    except:
      pass
    # raise NotImplementedError

# 2019-02-18 retryable, if failed first time, give up lock
class RetryableEvent(TryFailEvent):
  # https://english.stackexchange.com/a/305433
  def catch_exception(self, result_key, result):
    Event.catch_exception(self, result_key, result)
    if self.event_dispatch is None:
      return

    if self.get_id() not in self.event_dispatch.mutex_registry:
      return

    if not self.event_dispatch.mutex_registry[self.get_id()].locked():
      return

    print("RetryableEvent::catch_exception giving up event's lock")
    self.event_dispatch.mutex_registry[self.get_id()].release()

# some handy events commonly used
class IterateEvent(Event):
  def __init__(self, event_id, *args, **kwargs):
    super(IterateEvent, self).__init__(
      event_id, *args, **kwargs)

    self._exception = False

  def dispatch(self, event_dispatch, *args, **kwargs):
    super(IterateEvent, self).dispatch(
      event_dispatch, *args, **kwargs)

    # print("dispatching")
    iterable_object_key = args[0]

    try:
      # print("IterateEvent on %s" % (iterable_object_key))
      event_dispatch.blackboard[
        iterable_object_key].iterate(event_dispatch, self)
    except Exception as e:
      # print(e)
      # track = traceback.format_exc()
      # print(track)
      self._exception = True

    # print("done IterateEvent on %s" % (iterable_object_key))

  def finish(self, event_dispatch, *args, **kwargs):
    # ############### option A:
    # spawn the next event as an explicit
    # **child** thread
    # constructor_args = (self.event_id,
    #   event_dispatch.blackboard)
    # event_dispatch.dispatch(
    #   blackboard["IterateEvent"](
    #     *constructor_args), ())

    # ############### option B:
    # spawn the next event through the queue
    # and the ED, aka, a **sibling** thread
    # print("::finish")

    if self._exception:
      print("exception caught")
      return

    iterable_object_key = args[0]
    ed_prefix = args[1]

    event_dispatch.blackboard[ed_prefix + "_cv"].acquire()
    event_dispatch.blackboard[ed_prefix + "_queue"].append(
      [
        "IterateEvent",
        self.event_id,
        iterable_object_key,
        ed_prefix])
    event_dispatch.blackboard[ed_prefix + "_cv"].notify(1)
    event_dispatch.blackboard[ed_prefix + "_cv"].release()

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    tokens = args[0] # args is a tuple of 1 list, that list is tokens
    if len(tokens) != 3:
      raise Exception("expected 3 tokens")
      # event_id, iterable_object_key, ed_prefix
    return (int(tokens[0]), blackboard), (tokens[1], tokens[2]) # tuple

class CmdSetEvent(IterateEvent):
  def dispatch(self, event_dispatch, *args, **kwargs):
    # set the pupper's cmd
    # print("updating pupper cmd")
    # event_dispatch.blackboard[
    #   args[0]]._cmd = args[2]

    # print("CmdSetEvent on %s" % (args[0]))
    if len(args[2]) > 0:
      # print("len(args[2])", len(args[2]))
      event_dispatch.blackboard[
        args[0]].set_and_iterate(args[2][0], event_dispatch, self)
      # thread-safe write and iterate right away
      # to force controller

  def finish(self, event_dispatch, *args, **kwargs):
    # do not self-produce
    # assume there will be another PupperEvent
    # to drive the next iterate
    if self._exception:
      print("exception caught")
      return

    iterable_object_key = args[0]
    ed_prefix = args[1]

    args[2].pop()
    # print("len(args[2])", args[2])

    if len(args[2]) > 0:
      event_dispatch.blackboard[ed_prefix + "_cv"].acquire()
      event_dispatch.blackboard[ed_prefix + "_queue"].append(
        [
          "CmdSetEvent",
          self.event_id,
          iterable_object_key,
          ed_prefix,
          args[2]])
      event_dispatch.blackboard[ed_prefix + "_cv"].notify(1)
      event_dispatch.blackboard[ed_prefix + "_cv"].release()
    elif len(args[2]) == 0:
      event_dispatch.blackboard[
        args[0]].handle_lastcmdset(None, event_dispatch, self)

      event_dispatch.blackboard[ed_prefix + "_cv"].acquire()
      event_dispatch.blackboard[ed_prefix + "_queue"].append(
        [
          "IterateEvent",
          self.event_id,
          iterable_object_key,
          ed_prefix])
      event_dispatch.blackboard[ed_prefix + "_cv"].notify(1)
      event_dispatch.blackboard[ed_prefix + "_cv"].release()

  @staticmethod
  def deserialize(blackboard, *args, **kwargs):
    tokens = args[0] # args is a tuple of 1 list, that list is tokens
    if len(tokens) != 4:
      raise Exception("expected 4 token")
      # event_id, iterable_object_key, ed_prefix, cmd
    return (int(tokens[0]), blackboard), (tokens[1], tokens[2], tokens[3]) # tuple

# JQ3uqm
# An undefined activation, fire-immediately dispatch
# ################## other ways of activation:
# ROS1 services, queue/mutex/condition variable/semaphore, clock
# ################## other ways of dispatching:
# priority queue, round-robins, different events in the same thread
# related to OS-level schedulers / round-robins etc.
# ################## insight
# and you can think of these as another kind of 'actor'
# that capture 'dead data' off the 'blackboard'
# giving them the initial imperative
# ################## insight
# you can also think of ED classes as 'activators'
# that produce 'activation' to
# IterableObjects trees (temporally nested hierarchies)
# EventDispatch graphs (combinatorial dynamic graphs)
# ################## insight
# these relate the 'external' (clock / ros / blackboard)
# to the 'ingestion' point of a discrete time system
# IterableObjects tree root (temporal outer rim)
# EventDispatch nodes (any as long as it consumes)
# ################## insight
# EventDispatches are enzymes
# they *reduce* activation energy
# make imperatives (Events) inevitable
# ################## insight
# EventDispatches manifest the
# matrix perspective of *currency*
class EventDispatch(object):
  def __init__(self, blackboard = None, ed_id = None):
    self.thread_registry = {}
    self.mutex_registry = {}

    self.dispatch_mutex = Lock()
    self.dispatch_switch_mutex = Lock()
    self.dispatch_switch = True
    wrap_instance_method(self, 'dispatch',
      call_when_switch_turned_on(
        self, "dispatch_switch",
        "dispatch_switch_mutex")) # [example] decorator
    # safety mechanism:
    # if any event sets the switch off
    # no other events are dispatched
    # until the switch is cleared

    if (blackboard is not None and ed_id is not None):
      # give an ED a blackboard on which other EDs live
      # for when there is no ROS infrastructure for example
      self.blackboard = blackboard
      self.ed_id = ed_id
      self.blackboard[ed_id] = self # register self on blackboard

  def dispatch(self, event, *args, **kwargs):
    with self.dispatch_mutex:
      # print("dispatching %s" % (event.get_id())) # debug
      self.thread_registry[event.get_id()] = EventThread(
        target=lambda args = args, kwargs = kwargs:\
          event.dispatch(self, *args, **kwargs),
        # note that the event is dispatched with a reference
        # to the event dispatch, giving access / control
        # over other events
        callback=lambda event = event, args = args, kwargs = kwargs:\
          self.dispatch_finish(event, *args, **kwargs))
      self.thread_registry[event.get_id()].start()

  def dispatch_finish(self, event, *args, **kwargs):
    self.thread_registry.pop(event.get_id(), None)
    # self.log("finishing %s" % (event.get_id())) # debug
    event.finish(self, *args, **kwargs)
    # ONLY the event defines what is dispatched next
    # this includes multiple subsequent concurrent events

  def log(self, msg, params = None):
    # for children to override
    print(msg)

class ROS1EventDispatch(EventDispatch):
  import sys
  modulename = 'rospy'
  if modulename not in sys.modules:
    try:
      import rospy
    except Exception as e:
      print("failed importing rospy", str(e))
  def __init__(self, blackboard = None, ed_id = None,
    node_name = "",
    service_req_type = None,
    service_res_type = None,
    service_handler = None):
    super(ROS1EventDispatch, self).__init__(blackboard, ed_id)

    if blackboard is None:
      self.log(
        "EventDispatch::no blackboard => no service",
        "warn")
      return
    if node_name == "":
      self.log(
        "EventDispatch::no node_name => no service",
        "warn")
      return

    self.service_req_type = service_req_type
    self.service_res_type = service_res_type

    self.service_handler = self.event_dispatch_srv_cb # the default
    if service_handler is not None:
      self.service_handler = service_handler

    # have a service you can shut down the ED for example
    # over ROS even if you don't have a blackboard with Event definitions
    self.service_proxy = rospy.Service(
      "/%s/dispatch_event" % (node_name),
      self.service_req_type, # flexible about the msg type
      self.service_handler)

  def event_dispatch_srv_cb(self, req):
    self.log("EventDispatch received eventclass: %s" % (
      req.eventclass))

    # 2019-02-15 assume self.blackboard is not None
    # because otherwise service would not be up
    if req.eventclass not in self.blackboard.keys():
      self.log(
        "EventDispatch::event_dispatch_srv_cb no %s in blackboard" % (
        req.eventclass))
      return self.service_res_type(response="noop, not found")

    self.log("%s in blackboard, deserializing + dispatching" % (
      req.eventclass))

    try:
      constructor_args, dispatch_args = self.blackboard[
        req.eventclass].deserialize(self.blackboard, req)
    except Exception as e:
      return self.service_res_type(
        response="failed to deserialize %s" % (str(e)))

    # mechanism
    self.dispatch(
      self.blackboard[req.eventclass](*constructor_args),
      *dispatch_args)

    return self.service_res_type(response="dispatched")

  def log(self, msg, params = None):
    if params == "warn":
      rospy.logwarn(msg)
    else:
      rospy.loginfo(msg)

# run this in a thread
# ################## insight
# could be made to behave like a time-based loop
# if you had a timer producer that notified this
# thread at a fixed rate
# ################## insight
# whereas timer based activation is uniform in time
# this type of activation
# no busy-wait & non-uniform time spacing
# ################## type
# this is a SINGLE queue ED, not a compound queue
class BlackboardQueueCVED(EventDispatch):
  def __init__(self, blackboard, name):
    super(BlackboardQueueCVED, self).__init__(
      blackboard, name + "_dispatch")

    self.register_blackboard_assets(
      blackboard, name)

  def register_blackboard_assets(
    self, blackboard, name):
    self.name = name

    self.hb_key = name + "_hb"
    if self.hb_key not in blackboard:
      blackboard[self.hb_key] = True
    # assert(self.hb_key in blackboard)

    self.mutex_name = name + "_mutex"
    if self.mutex_name not in blackboard:
      # without creating one explicitly
      # condition has underlying mutex
      blackboard[self.mutex_name] = Lock()
    # assert(self.mutex_name in blackboard)

    self.cv_name = name + "_cv"
    if self.cv_name not in blackboard:
      blackboard[self.cv_name] = Condition(
        blackboard[self.mutex_name])
    # assert(self.cv_name in blackboard)

    self.queue_name = name + "_queue"
    if self.queue_name not in blackboard:
      blackboard[self.queue_name] = []
    # assert(self.queue_name in blackboard)

  def run(self,
    blackboard, # expected, dict
    prefix, # expected, str
    empty_cv_name = None, # expected, str
    debug_color = None):
    assert(blackboard is not None)

    if (debug_color is not None):
      wrap_instance_method(self, 'log',
        wrap_with_prints(debug_color, bcolors.ENDC))
      # [example] decorator

    while(blackboard[self.hb_key]):
      blackboard[self.cv_name].acquire()
      # syntax in while bool expression (cv predicate) is key
      while blackboard[self.hb_key] and (
        len(blackboard[self.queue_name]) == 0):
        blackboard[self.cv_name].wait()
        # Wait until notified or until a timeout occurs.
        # If the calling thread has not acquired the lock
        # when this method is called,
        # a RuntimeError is raised.
        # add conditions (predicates) to protect
        # against spurious wakeups prior or after
        # condition is actually met
      blackboard[self.cv_name].release()

      # could be woken from shutdown procedure
      if len(blackboard[self.queue_name]) == 0:
        self.log("woken from shutdown")
        break

      s = blackboard[self.queue_name].pop(0)

      ##### core ED logic ####################################
      # s (serialized_event) expected to be
      # array of [<class>, args]
      if len(s) >= 1:
        # deserialize & dispatch
        # print("s[0]", s[0])
        try:
          constructor_args, dispatch_args = blackboard[
            s[0]].deserialize(
              blackboard, s[1:])
          # mechanism
          self.dispatch(
            blackboard[s[0]](*constructor_args),
            *dispatch_args)
        except Exception as e:
          self.log(self.ed_id
            + " failed dispatch %s, exception %s" % (
              str(s), str(e)))
      ########################################################

      # ED tries to 'cleanup'
      if empty_cv_name is not None:
        if len(blackboard[self.queue_name]) == 0 and\
          empty_cv_name in blackboard:
          self.log("notifying " + empty_cv_name)
          blackboard[empty_cv_name].acquire()
          blackboard[empty_cv_name].notify_all()
          blackboard[empty_cv_name].release()

    self.log(self.name + " shutdown")

  def log(self, msg, params = None):
    print(msg)

# 2019-01-08 https://stackoverflow.com/a/3387975
# python dicts are lock protected but this is more explicit
# 2019-01-09 a blackboard is a tool to organize data spatially together
# you can have many different blackboards encompassing different areas
# as well as nesting, define spatial and temporal scope in events / dispatching
# anything that is 'static time' / state goes here
class Blackboard(collections.MutableMapping):
  def __init__(self, *args, **kwargs):
    self.mutex = Lock()
    self.store = dict()
    self.update(dict(*args, **kwargs))  # use the free update to set keys

    # guarantee the lock is released on failures
    wrap_instance_method(self,
      "__getitem__",
      watchdog_try_to_run_and_notify_failure(
        "__getitem__",
        None,
        self.catch_exception))

    wrap_instance_method(self,
      "__setitem__",
      watchdog_try_to_run_and_notify_failure(
        "__setitem__",
        None,
        self.catch_exception))

    wrap_instance_method(self,
      "__delitem__",
      watchdog_try_to_run_and_notify_failure(
        "__delitem__",
        None,
        self.catch_exception))

  def catch_exception(self, result_key, result):
    self.mutex.release()

  def __getitem__(self, key):
    self.mutex.acquire()
    if self.__keytransform__(key) not in self.store:
      self.mutex.release()
      return None
    x = self.store[self.__keytransform__(key)]
    self.mutex.release()
    return x

  def __setitem__(self, key, value):
    self.mutex.acquire()
    self.store[self.__keytransform__(key)] = value
    self.mutex.release()

  def __delitem__(self, key):
    self.mutex.acquire()
    if self.__keytransform__(key) not in self.store:
      self.mutex.release()
      return
    del self.store[self.__keytransform__(key)]
    self.mutex.release()

  def __iter__(self):
    return iter(self.store)

  def __len__(self):
    return len(self.store)

  def __keytransform__(self, key):
    return key

# base class for this kind of 'actor'
# that relates to other IterableObjects through
# post-order traversal parentage
class IterableObject(object):
  def __init__(self, name = None):
    self._initialized = False

    self._name = name

    self._parent = None
    self._children = []
    # other IterableObjects
    # visited in a post-order traversal fashion

  def init(self, *args, **kwargs):
    self._children.clear()

    if (self._initialized):
      raise Exception(
        "cannot init if already initialized")

    try:
      self.do_init(*args, **kwargs)
    except Exception as e:
      print(str(e))
      return

    for child in self._children:
      if child == self:
        raise Exception("child == parent")

      child.init(*args, **kwargs)

    self._initialized = True

  def initialized(self):
    return self._initialized

  def add_child(self, child):
    if child == self:
      raise Exception("child == self")

    for c in self._children:
      if child == c:
        raise Exception("child == c")

    child._parent = self
    self._children.append(child)

  def iterate(self, *args, **kwargs):
    if not self._initialized:
      raise Exception(
        "not initialized")

    # post-order graph traversal
    for child in self._children:
      if (child == self):
        raise Exception("child == self")

      child.iterate(*args, **kwargs)

    self.do_iterate(*args, **kwargs)

  def cleanup(self):
    self.do_cleanup()

    for child in self._children:
      if (child == self):
        raise Exception("child == self")

    self._initialized = False

  # for child classes to override
  # throw exception to interrupt
  def do_init(self, *args, **kwargs):
    raise NotImplementedError

  def do_iterate(self, *args, **kwargs):
    raise NotImplementedError

  def do_cleanup(self):
    raise NotImplementedError
