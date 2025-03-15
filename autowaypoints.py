from commands.jerky_trajectory import mirror

# waypoints for autos

class SideE:
    kEndpoint = {
        "right": (5.155, 5.899, -120),
        "left": (5.906, 5.919, -120),
    }

    kApproach = [
        (6.491, 5.846, -170.0),
    ]

    kReload = [
        (4.755, 6.199, -120),
        (2.685, 5.515, -54.0),
        (2.385, 5.815, -54.0),
    ]


class SideDLeft:
    kEndpoint = {
        "left": (6.89, 3.80, 180),
        "right": (6.89, 4.25, 180),
    }

    kApproach = []

    kReload = [
        (6.791, 4.832, -180.0),
        (5.291, 6.632, -180.0),
        (2.685, 5.515, -54.0),
        (2.385, 5.815, -54.0),
    ]


class SideDRight:
    kApproach = mirror(SideDLeft.kApproach)
    kReload = mirror(SideDLeft.kReload)
    kEndpoint = {
        "right": mirror(SideDLeft.kEndpoint["left"]),
        "left": mirror(SideDLeft.kEndpoint["right"]),
    }


class SideC:
    kApproach = mirror(SideE.kApproach)
    kReload = mirror(SideE.kReload)
    kEndpoint = {
        "right": mirror(SideE.kEndpoint["left"]),
        "left": mirror(SideE.kEndpoint["right"]),
    }
