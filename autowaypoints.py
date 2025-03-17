from commands.jerky_trajectory import mirror

# waypoints for autos

class SideE:
    kEndpoint = {
        "right": (5.655, 5.999, -120),
        "left": (5.906, 5.879, -120),
    }

    tags = (11, 20)  # tags of side E

    kApproach = [
        (6.305, 6.099, -120),
    ]

    kReload = [
        (4.755, 6.099, -45),
        (3.755, 6.099, -45),
#        (3.285, 6.05, -30.0),
        (2.685, 6.015, -35.0),
    ]


class SideDLeft:
    kEndpoint = {
        "left": (6.09, 3.84, 180),
        "right": (6.09, 4.21, 180),
    }

    tags = (10, 21)  # tags of side D

    kApproach = []

    kReload = [
        (6.791, 4.832, -90.0),
        (6.791, 5.432, -90.0),
        (4.791, 6.632, -30.0),
        (3.185, 5.815, -54.0),
        (2.385, 5.815, -54.0),
    ]


class SideDRight:
    kApproach = mirror(SideDLeft.kApproach)
    kReload = mirror(SideDLeft.kReload)
    tags = SideDLeft.tags  # tags of side D
    kEndpoint = {
        "right": mirror(SideDLeft.kEndpoint["left"]),
        "left": mirror(SideDLeft.kEndpoint["right"]),
    }


class SideC:
    kApproach = mirror(SideE.kApproach)
    tags = (9, 22)  # tags of side C
    kReload = mirror(SideE.kReload)
    kEndpoint = {
        "right": mirror(SideE.kEndpoint["left"]),
        "left": mirror(SideE.kEndpoint["right"]),
    }
