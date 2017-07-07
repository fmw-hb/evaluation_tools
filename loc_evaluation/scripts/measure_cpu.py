#!/usr/bin/env python
import sys
import rospy

import psutil
import os

from optparse import OptionParser

def getProcessIDs(name):
    proc_id = []
    proc_name = []
    for proc in psutil.process_iter():
        try:
            if name in proc.name:
                print "process ", name, " id: ",proc.pid
                proc_id.append(proc.pid)
                proc_name.append(proc.name)
        except psutil.NoSuchProcess:
            pass

    return proc_id, proc_name

if __name__ == '__main__':
    rospy.init_node('measure_cpu', anonymous=True)

    parser = OptionParser(usage="%prog -p or --process + process name", prog=os.path.basename(sys.argv[0]))

    parser.add_option("-p", "--process", dest="process", default=0, help="Process name")

    (options, args) = parser.parse_args()
    if not isinstance(options.process, basestring):
        parser.error("Please specify a process name using -p or --process")
        sys.exit()
    process_IDs = []
    process_names = []
    process_IDs, process_names = getProcessIDs(options.process)
    if len(process_IDs) == 0:
        parser.error("No process named " + options.process)
    else:
        print "looking after CPU percentage of Processes " + str(process_names) + " with ID " + str(process_IDs)

    processes = []
    for pid in process_IDs:
        processes.append(psutil.Process(pid))

    count = 0
    cpu = 0
    mem = 0
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()
        for process in processes:
            cpu = cpu + process.get_cpu_percent()
            mem = mem + process.get_memory_percent()
        count = count + 1
        if (count%10 == 0):
            print "average CPU percentage is: " + str(cpu/count)
            print "average Memory percentage is: " + str(mem/count)


