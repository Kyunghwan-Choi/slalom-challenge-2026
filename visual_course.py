"""Optional live PyChrono display for the public practice runner.

All marker bodies are fixed, visual only, and excluded from contact testing.
They do not change the evaluator's vehicle, road, or cone dynamics.
"""

import time


class CourseVisual:
    def __init__(self, car, ground, cone_bodies, scenario):
        import pychrono as c
        import pychrono.vehicle as v

        self._chrono = c
        self._scenario = scenario
        system = car.GetSystem()
        marker_material = c.ChContactMaterialSMC()

        def box(length, width, height, x, y, z, color):
            body = c.ChBodyEasyBox(length, width, height, 100, True, False, marker_material)
            body.SetFixed(True)
            body.SetPos(c.ChVector3d(x, y, z))
            body.GetVisualShape(0).SetColor(c.ChColor(*color))
            system.AddBody(body)
            return body

        # A checkered ground stripe and two blue edge posts mark X = finish_x_m.
        # Every box has collision disabled, so this is a display cue only.
        finish_x = scenario["finish_x_m"]
        half_width = scenario["road_half_width_m"]
        self._markers = []
        n_lanes = 10
        tile_width = 2 * half_width / n_lanes
        for row in range(n_lanes):
            y = -half_width + (row + 0.5) * tile_width
            for column in range(2):
                x = finish_x + (column - 0.5) * 0.3
                color = (0.98, 0.98, 0.96) if (row + column) % 2 else (0.04, 0.10, 0.18)
                self._markers.append(box(0.3, tile_width, 0.012, x, y, 0.025, color))
        for side in (-1, 1):
            self._markers.append(
                box(0.12, 0.12, 2.6, finish_x, side * (half_width + 0.08), 1.3, (0.12, 0.44, 0.9))
            )
        self._markers.append(
            box(0.12, 2 * half_width + 0.28, 0.14, finish_x, 0, 2.65, (0.12, 0.44, 0.9))
        )

        ground.SetColor(c.ChColor(0.65, 0.70, 0.75))
        for cone in cone_bodies:
            cone.GetVisualShape(0).SetColor(c.ChColor(1, 0.28, 0.015))
        car.SetChassisVisualizationType(c.VisualizationType_MESH)
        car.SetWheelVisualizationType(c.VisualizationType_MESH)
        car.SetTireVisualizationType(c.VisualizationType_MESH)

        visual = v.ChWheeledVehicleVisualSystemIrrlicht()
        visual.SetWindowTitle("Slalom Challenge | live PyChrono | close window to stop")
        visual.SetWindowSize(1280, 800)
        visual.SetChaseCamera(c.ChVector3d(0, 0, 1.4), 9, 0.8)
        visual.Initialize()
        visual.AddLightDirectional()
        visual.AddSkyBox()
        visual.AttachVehicle(car.GetVehicle())
        self._visual = visual
        self._started = time.perf_counter()

    def synchronize(self, t, inputs):
        self._visual.Synchronize(t, inputs)

    def advance(self, dt):
        self._visual.Advance(dt)

    def draw(self, elapsed_s, gates_passed):
        if not self._visual.Run():
            return False
        self._visual.SetWindowTitle(
            f"Slalom Challenge | t={elapsed_s:.1f} s | gates={gates_passed}/{len(self._scenario['cones'])} | "
            f"finish X={self._scenario['finish_x_m']:g} m"
        )
        self._visual.BeginScene()
        self._visual.Render()
        self._visual.EndScene()
        remaining = elapsed_s - (time.perf_counter() - self._started)
        if remaining > 0:
            time.sleep(remaining)
        return True

    def show_result(self, status, finish_time_s):
        if status == "window_closed":
            return
        label = (
            f"FINISH in {finish_time_s:.2f} s" if status == "success" else status.replace("_", " ")
        )
        self._visual.SetWindowTitle(f"Slalom Challenge | {label} | close window to exit")
        for _ in range(40):
            if not self._visual.Run():
                break
            self._visual.BeginScene()
            self._visual.Render()
            self._visual.EndScene()
            time.sleep(0.04)

    def close(self):
        self._visual.GetDevice().closeDevice()
