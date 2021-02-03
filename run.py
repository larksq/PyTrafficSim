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

# DEMO 1: no left-turning light intersection
# test_is = IntersectionSim(win=win, trajectory=True)
# test_is.loop(agents_per_second=2.0, running_time=50)

# DEMO 2: with a left-turning light at demo intersection
test_is = IntersectionSim(win=win,
                          trajectory=False,
                          map_cls=DefaultMap_4ways_WithLeftTurn.Map,
                          spawn_vertical_margin=2.0,
                          collision_speculate_skip_rate=40,
                          speed_decrease_when_crowded=1.2)
test_is.loop(agents_per_second=9999.0, running_time=50)

win.autoflush = True
win.getMouse()
win.close()

