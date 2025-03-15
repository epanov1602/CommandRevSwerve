from commands.jerky_trajectory import mirror

# waypoints for autos

class SideE:
    kEndpoint = {
        "right": (5.655, 5.899, -120),
        "left": (6.106, 5.719, -120),
    }

    tags = (11, 20)  # tags of side E

    kApproach = [
#        (6.491, 5.846, -150.0),
    ]

    kReload = [
        (4.755, 6.299, -120),
        (2.985, 5.815, -30.0),
        (2.685, 6.015, -30.0),
    ]


class SideDLeft:
    kEndpoint = {
        "left": (6.09, 3.84, 180),
        "right": (6.09, 4.21, 180),
    }

    tags = (10, 21)  # tags of side D

    kApproach = []

    kReload = [
        (6.791, 4.832, -180.0),
        (6.991, 5.432, -180.0),
        (5.291, 6.632, -180.0),
        (3.185, 5.615, -54.0),
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
