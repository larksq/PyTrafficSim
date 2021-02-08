from PyTrafficSim import *
import getopt, sys

# 1. run dynamic testing
# the scene has 1000*1000px, 0.1m/px
# window_h = 1000
# window_w = window_h
# win = GraphWin('DynamicTest_PTS', window_h, window_w)
# win.autoflush = False          # NEW: set before animation
#
# test = DynamicTest(win=win, window_h=window_h, window_w=window_w)
# test.runLoop()
#
# win.autoflush = True          # NEW: set before animation
# win.getMouse()
# win.close()



# 2. run intersection simulation
default_demo_id = 1
argumentList = sys.argv[1:]
options = "d:"
long_options = ["Demo="]
demo_id = default_demo_id

try:
    arguments, values = getopt.getopt(argumentList, options, long_options)
    for currentArgument, currentValue in arguments:
        if currentArgument in ["-d", "--Demo"]:
            if currentValue.isnumeric() and int(currentValue) in [1, 2, 3]:
                demo_id = int(currentValue)
            else:
                print("Ending with illegal demo id, choose a number from 1-3")
                exit()
        else:
            print("Ending with Unknown Commands")
            exit()
except getopt.error as err:
    # output error, and return with an error code
    print("Running default demo: ", str(default_demo_id))

window_h = 1000
window_w = window_h
win = GraphWin('PyTrafficSim', window_h, window_w)
win.autoflush = False

if demo_id == 1:
    # DEMO 1: no left-turning light intersection
    test_is = IntersectionSim(win=win, trajectory=True)
    test_is.loop(agents_per_second=1.2, running_time=50)
elif demo_id == 2:
    # DEMO 2: a left-turning light at demo intersection
    test_is = IntersectionSim(win=win,
                              trajectory=False,
                              map_cls=DefaultMap_4ways_WithLeftTurn.Map,
                              spawn_vertical_margin=5.0,
                              collision_speculate_skip_rate=40,
                              speed_decrease_when_crowded=1.3,
                              extend=20)
    test_is.loop(agents_per_second=9999.0, running_time=50)
elif demo_id == 3:
    # DEMO 3: 4 ways stop sign at demo intersection
    test_is = IntersectionSim(win=win, trajectory=True, map_cls=DefaultMap_4ways_StopSigns.Map,
                              collision_speculate_skip_rate=10, initial_speed_rate=0.6, speed_decrease_when_crowded=1.3,
                              cruising_speed_rate=0.7)
    test_is.loop(agents_per_second=0.5, running_time=40)

# DEMO x: load data from the NuScene dataset (undone)
# test_is = SimWithLoadedData(win=win,
#                             trajectory=False,
#                             collision_speculate_skip_rate=40,
#                             scale=5)
# test_is.loop_with_loaded_data()


win.autoflush = True
win.getMouse()
win.close()
