from PyTrafficSim import *

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
window_h = 1000
window_w = window_h
win = GraphWin('PyTrafficSim', window_h, window_w)
win.autoflush = False

test_is = IntersectionSim(win=win, trajectory=False)
test_is.loop(agents_per_second=2.0, running_time=50)

win.autoflush = True
win.getMouse()
win.close()

