# iPhone 16 Pro chest holder (POV, portrait) - twist-lock fabric mount
# Fusion > Utilities > Scripts and Add-Ins > (+) > select this file > Run
# Coordinates: X = phone width, Y = phone height (top = camera), +Z = outward (phone back/camera side)
import adsk.core, adsk.fusion, math, traceback

mm = 0.1


def P(x, y, z=0):
    return adsk.core.Point3D.create(x * mm, y * mm, z * mm)


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        root = design.rootComponent
        exts = root.features.extrudeFeatures
        XY = root.xYConstructionPlane
        NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
        CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

        # ---- parameters (mm) ----
        PW, PH, PT = 71.5, 149.6, 8.25      # iPhone 16 Pro (bare, no case)
        C = 0.5                              # side clearance
        WALL, BASE, LIPT, LIPW = 2.5, 2.5, 2.0, 4.0
        IW = PW + 2 * C
        HW = IW / 2
        PD = PT + 0.6                        # pocket depth
        ZP0 = BASE
        ZP1 = BASE + PD
        ZTOP = ZP1 + LIPT
        OX = HW + WALL
        YTOP = PH + C + 3.5 + 2.5            # extra room above phone for lock pin
        PINY = PH + C + 1.7
        PINZ = (ZP0 + ZP1) / 2
        SCY = 75.05                          # socket centre (phone centre)

        def rrect(sk, x0, y0, x1, y1, r):
            L = sk.sketchCurves.sketchLines
            A = sk.sketchCurves.sketchArcs
            L.addByTwoPoints(P(x0 + r, y0), P(x1 - r, y0))
            L.addByTwoPoints(P(x1, y0 + r), P(x1, y1 - r))
            L.addByTwoPoints(P(x1 - r, y1), P(x0 + r, y1))
            L.addByTwoPoints(P(x0, y1 - r), P(x0, y0 + r))
            A.addByCenterStartSweep(P(x1 - r, y0 + r), P(x1 - r, y0), math.pi / 2)
            A.addByCenterStartSweep(P(x1 - r, y1 - r), P(x1, y1 - r), math.pi / 2)
            A.addByCenterStartSweep(P(x0 + r, y1 - r), P(x0 + r, y1), math.pi / 2)
            A.addByCenterStartSweep(P(x0 + r, y0 + r), P(x0, y0 + r), math.pi / 2)

        def rect(sk, x0, y0, x1, y1):
            sk.sketchCurves.sketchLines.addTwoPointRectangle(P(x0, y0), P(x1, y1))

        def ext(sk, z0, z1, op, name, body=None):
            profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
            prof = max(profs, key=lambda p: p.areaProperties().area)
            inp = exts.createInput(prof, op)
            inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(z0 * mm))
            inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z1 - z0) * mm))
            if body is not None:
                inp.participantBodies = [body]
            f = exts.add(inp)
            f.name = name
            return f

        def newsk(name):
            s = root.sketches.add(XY)
            s.name = name
            return s

        # 1 outer shell
        s = newsk('Outer'); rrect(s, -OX, -WALL, OX, YTOP, 10)
        body = ext(s, 0, ZTOP, NEW, 'Shell').bodies.item(0)
        body.name = 'Holder'
        # 2 phone pocket (open at top for slide-in)
        s = newsk('Pocket'); rrect(s, -HW, 0, HW, YTOP + 20, 8)
        ext(s, ZP0, ZP1, CUT, 'Pocket', body)
        # 3 retaining lip window
        s = newsk('LipWindow'); rrect(s, -(HW - LIPW), LIPW, HW - LIPW, YTOP + 20, 6)
        ext(s, ZP1 - 0.1, ZTOP + 1, CUT, 'LipWindow', body)
        # 4 camera plateau clearance (top-left seen from outside)
        s = newsk('CameraClear'); rect(s, -OX - 5, PH - 55, -(HW - LIPW) + 1, YTOP + 20)
        ext(s, ZP1 - 0.1, ZTOP + 1, CUT, 'CameraClear', body)
        # 5 bottom USB-C / speaker opening
        s = newsk('BottomPort'); rect(s, -25, -WALL - 5, 25, 1)
        ext(s, ZP0, ZP1, CUT, 'BottomPort', body)
        # 6 button slots (+X: action/volume, -X: side button + Camera Control)
        for nm, x0, x1, a, b in [('VolSlot', HW - LIPW - 1, OX + 5, 22, 78),
                                 ('SideBtnSlot', -OX - 5, -(HW - LIPW) + 1, 32, 62),
                                 ('CamCtrlSlot', -OX - 5, -(HW - LIPW) + 1, 82, 115)]:
            s = newsk(nm); rect(s, x0, PH - b, x1, PH - a)
            ext(s, ZP0 + 1, ZTOP + 1, CUT, nm, body)
        # 7 lightening windows in base plate
        for nm, y0, y1 in [('WinBottom', 10, 42), ('WinTop', 108, 140)]:
            s = newsk(nm); rrect(s, -24, y0, 24, y1, 5)
            ext(s, -0.1, BASE + 0.1, CUT, nm, body)
        # 8 twist-lock socket on chest side
        s = newsk('SocketBoss'); s.sketchCurves.sketchCircles.addByCenterRadius(P(0, SCY), 28 * mm)
        ext(s, -7, 0, JOIN, 'SocketBoss', body)
        s = newsk('SocketRecess'); s.sketchCurves.sketchCircles.addByCenterRadius(P(0, SCY), 23.5 * mm)
        ext(s, -7.1, 0.5, CUT, 'SocketRecess', body)   # floor 2 mm, room for head + fabric
        s = newsk('SocketLugs')
        for ang in [90, 210, 330]:
            a0 = math.radians(ang - 20); a1 = math.radians(ang + 20); r0, r1 = 20, 24
            pts = [P(r * math.cos(a), SCY + r * math.sin(a)) for r, a in [(r0, a0), (r1, a0), (r1, a1), (r0, a1)]]
            L = s.sketchCurves.sketchLines; A = s.sketchCurves.sketchArcs
            L.addByTwoPoints(pts[0], pts[1]); L.addByTwoPoints(pts[2], pts[3])
            A.addByCenterStartSweep(P(0, SCY), pts[1], math.radians(40))
            A.addByCenterStartSweep(P(0, SCY), pts[0], math.radians(40))
        profs = adsk.core.ObjectCollection.create()
        for i in range(s.profiles.count):
            profs.add(s.profiles.item(i))
        inp = exts.createInput(profs, JOIN)
        inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(-7 * mm))
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2.5 * mm))
        inp.participantBodies = [body]
        exts.add(inp).name = 'SocketLugs'
        # 9 top lock-pin hole (dia 3.3, for M3 bolt or printed pin) along X
        s = root.sketches.add(root.yZConstructionPlane); s.name = 'PinHole'
        s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(0, PINY, PINZ)), 1.65 * mm)
        inp = exts.createInput(s.profiles.item(0), CUT)
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(5), True)
        inp.participantBodies = [body]
        exts.add(inp).name = 'PinHole'

        # ===== Inner disc (worn inside the shirt; fabric is pinched between disc and holder) =====
        TF = 0.8                       # fabric allowance per layer
        ZF1 = -7 - TF; ZF0 = ZF1 - 2.5 # flange
        ZH0 = -4.5 + TF; ZH1 = ZH0 + 3.0  # head lugs
        RC, RL = 18.8, 22.3            # post radius, lug outer radius
        C0 = 87                        # shown in locked pose (insert at ~27 deg, twist +60)

        def sector(sk, r0, r1, d0, d1):
            a0 = math.radians(d0); a1 = math.radians(d1)
            p = [P(r * math.cos(a), SCY + r * math.sin(a)) for r, a in [(r0, a0), (r1, a0), (r1, a1), (r0, a1)]]
            L = sk.sketchCurves.sketchLines; A = sk.sketchCurves.sketchArcs
            L.addByTwoPoints(p[0], p[1]); L.addByTwoPoints(p[2], p[3])
            A.addByCenterStartSweep(P(0, SCY), p[1], a1 - a0)
            A.addByCenterStartSweep(P(0, SCY), p[0], a1 - a0)

        def ext_all(sk, z0, z1, op, name, bd):
            c = adsk.core.ObjectCollection.create()
            for i in range(sk.profiles.count):
                c.add(sk.profiles.item(i))
            inp = exts.createInput(c, op)
            inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(z0 * mm))
            inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z1 - z0) * mm))
            inp.participantBodies = [bd]
            exts.add(inp).name = name

        s = newsk('DiscFlange'); s.sketchCurves.sketchCircles.addByCenterRadius(P(0, SCY), 30 * mm)
        disc = ext(s, ZF0, ZF1, NEW, 'DiscFlange').bodies.item(0)
        disc.name = 'InnerDisc'
        s = newsk('DiscPost'); s.sketchCurves.sketchCircles.addByCenterRadius(P(0, SCY), RC * mm)
        ext(s, ZF1 - 0.1, ZH1, JOIN, 'DiscPost', disc)
        s = newsk('DiscLugs')
        for k in range(3):
            sector(s, RC - 0.5, RL, C0 + 120 * k - 18, C0 + 120 * k + 18)
        ext_all(s, ZH0, ZH1, JOIN, 'DiscLugs', disc)
        s = newsk('DiscStops')         # rotation stops at trailing end of each lug
        for k in range(3):
            sector(s, RC - 0.5, RL, C0 + 120 * k - 24, C0 + 120 * k - 17.5)
        ext_all(s, ZH0 - 2.0, ZH0 + 0.1, JOIN, 'DiscStops', disc)
        disc = root.bRepBodies.itemByName('InnerDisc')
        edges = adsk.core.ObjectCollection.create()
        for e in disc.edges:
            g = e.geometry
            if isinstance(g, adsk.core.Circle3D) and abs(g.radius * 10 - 30) < 0.01:
                edges.add(e)
        fi = root.features.filletFeatures.createInput()
        fi.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(0.1), True)
        root.features.filletFeatures.add(fi)
        disc = root.bRepBodies.itemByName('InnerDisc')
        edges = adsk.core.ObjectCollection.create()
        for e in disc.edges:
            bb = e.boundingBox
            if abs(bb.minPoint.z * 10 - ZH1) < 0.01 and abs(bb.maxPoint.z * 10 - ZH1) < 0.01:
                edges.add(e)
        fi = root.features.filletFeatures.createInput()
        fi.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(0.06), True)
        root.features.filletFeatures.add(fi)

        app.activeViewport.fit()
    except Exception:
        ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
