import swerveutils

def test_stepTowardsCircular1():
    current = 0.6408134451373411
    stepsize = 0.3455804605358387
    target = 0.0  # stepping towards zero direction
    result = swerveutils.stepTowardsCircular(current=current, stepsize=stepsize, target=target)
    assert abs(result) < abs(current), f"swerveutils test: why stepping from {current} to {target} results in {result}?"
