#!/usr/bin/env python3

import serial, sys, time

from common import IterableObject, Blackboard

class SerialBridge(IterableObject):
  def __init__(self):
    super(SerialBridge, self).__init__()

    self._serial_handle = None

    self._blackboard = Blackboard()

    # character buffer array
    # that children update
    # and iterate resets
    self._blackboard["tx_buffer"] = ""
    self._blackboard["rx_buffer"] = ""

  def do_init(self, *args, **kwargs):
    self._serial_handle = serial.Serial(
      args[0],
      args[1],
      timeout = 1.0) # baudrate is key

    self._serial_handle.close(); self._serial_handle.open()
    # https://stackoverflow.com/a/38495159/13139390
    time.sleep(2)

    self.external_blackboard = None
    self.init_external_blackboard(*args)

  def init_external_blackboard(self, *args):
    # for child classes to override

    # base class
    # expect args[0] to be dict
    # exposes 1 str
    # and sets member var
    if type(args[2]) is not dict:
      raise Exception("external_blackboard not dict")
    self.external_blackboard = args[2]
    self.external_blackboard["rx_buffer"] = ""

  def produce(self, data):
    # for child classes to override

    # base class is pass-through to buffer
    self.external_blackboard["rx_buffer"] = data
    print(data)

  def teardown(self, blackboard):
    # for child classes to override

    blackboard["done"].acquire()
    blackboard["done_queue"].clear()
    blackboard["done"].notify_all()
    blackboard["done"].release()

    blackboard["sb"]._serial_handle.close()

  def do_iterate(self, *args, **kwargs):
    if len(self._blackboard["tx_buffer"]) > 0:
      self._serial_handle.write(
        str.encode(self._blackboard["tx_buffer"]))
      self._blackboard["tx_buffer"] = ""

    read_data = self._serial_handle.readline()
    read_data = read_data.decode('ascii').rstrip('\r\n')

    if len(read_data) > 0:
      if read_data == "end":
        self.teardown(args[0].blackboard)
        return

      self.produce(read_data)

  def do_cleanup(self):
    try:
      self._serial_handle.close()
    except Exception as e:
      pass

if __name__ == '__main__':
  from common import bcolors, nonempty_queue_exists
  from common import IterateEvent, BlackboardQueueCVED

  import argparse
  from threading import Lock, Condition, Thread

  parser = argparse.ArgumentParser(
    description="")
  parser.add_argument('--serial', type=str,
    default="/dev/ttyACM0")
  parser.add_argument('--baudrate', type=int,
    default=38400)
  args = parser.parse_args()

  ############### overhead
  blackboard = {}

  # we purposely use the 'done' set to keep
  # this process alive, and the done_queue
  # being non-empty until some actor / event
  # pop's it empty and notifies 'done'
  # effectively killing the program
  blackboard["done"] = Condition()
  blackboard["done_mutex"] = Lock()
  blackboard["done_queue"] = [1]

  ############### actors
  sb = SerialBridge()
  sb.init(args.serial, args.baudrate, blackboard)
  if not sb.initialized():
    print("couldn't initialize sb")
    sys.exit(1)
  blackboard["sb"] = sb

  ############### dispatches
  controller_ed = BlackboardQueueCVED(
    blackboard, "sb")
  blackboard["sb_thread"] = Thread(
    target=controller_ed.run,
    args=(blackboard,
      "sb",
      # "done",
      None,
      bcolors.CYAN))

  ############### events
  blackboard["IterateEvent"] = IterateEvent

  ############### process init
  blackboard["sb_cv"].acquire()
  blackboard["sb_queue"].append(
    ["IterateEvent", 1, "sb", "sb"])
  blackboard["sb_cv"].notify(1)
  blackboard["sb_cv"].release()

  ############### process lifecycle
  blackboard["sb_thread"].start()

  # really it is wait on a Condition that
  # all consumers notify
  # but predicate on all queues being empty
  blackboard["done"].acquire()
  while nonempty_queue_exists(blackboard,
    [
      # queue names that are admissible
      # if they are nonempty
    ],
    # verbose = True
    ):
    # ) or not blackboard["atomic_bool"]:
    blackboard["done"].wait()
  blackboard["done"].release()

  ### Shutdown procedure
  print("### SHUT DOWN ###")

  # purge all shared resources
  # set all heartbeat(hb)s to false
  for k in blackboard.keys():
    if k[-3:] != "_hb":
      continue
    print("notifying hb %s" % (k))
    blackboard[k] = False

  # notify all condition variables
  for k in blackboard.keys():
    if k[-3:] != "_cv":
      continue
    print("notifying cv %s" % (k))
    mutex_k = k[:-3] + "_mutex"
    blackboard[mutex_k].acquire()
    blackboard[k].notify_all()
    blackboard[mutex_k].release()

  # join threads
  for k in blackboard.keys():
    if k[-7:] != "_thread":
      continue
    print("joining", k)
    blackboard[k].join()

  sb.cleanup()
