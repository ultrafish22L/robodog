# v10 pre-commit gate (no FreeCAD): lock trunk-height TH from servo packaging + the
# measured 1:1.7 thigh:tibia ratio, then check STATIC hold torque at hip-pitch & knee
# vs SG90 (1.8) / MG90S (2.2) kg-cm. Direct-drive knee (option 1).
SG90 = (22.7, 12.1, 22.5)          # L,W,H mm
STALL_SG90, STALL_MG90S = 1.8, 2.2 # kg-cm

# Packaging: the femur (thigh) hosts the hip-pitch clevis (proximal) + the knee-servo
# cradle (distal); they must not overlap. Numbers from the verified leg4 geometry.
KNEE_SERVO_REACH = 17.0   # knee servo body reaches X=-17 from the knee axis (box 23 long)
CLEVIS_REACH     = 15.0   # hip-pitch rounded-knuckle clevis extent at the femur proximal
GAP              = 6.0    # clearance so the two servo bodies clear
femur_min = KNEE_SERVO_REACH + GAP + CLEVIS_REACH
print("femur_min (servo packaging) = %.0f mm" % femur_min)

RATIO = 1.7               # measured thigh:tibia = 1:1.7 (tibia is the long bone)
for femur in (femur_min, 46.0, 52.0):
    tibia = RATIO * femur
    TH = femur / 0.71                 # thigh = 0.71*TH  ->  TH from femur
    hipH = 1.78 * TH
    foot_ahead = 0.36 * TH            # foot lands this far AHEAD of its hip (horizontal)
    print("\nfemur=%.0f tibia=%.0f leg=%.0f | TH=%.0f body=%.0fx%.0fx%.0fmm hipH=%.0f foot_ahead=%.1f"
          % (femur, tibia, femur+tibia, TH, 3.7*TH, 2.0*TH, TH, hipH, foot_ahead))
    for total_g in (300, 400, 500):
        legload = total_g/4.0/1000.0                  # kg vertical per leg, standing on 4
        tau_hip = legload * (foot_ahead/10.0)         # hip-pitch arm = horiz foot offset (cm)
        knee_arm = abs(foot_ahead - foot_ahead*0.45)/10.0   # horiz foot->knee (cm), est.
        tau_knee = legload * knee_arm
        print("   %dg total (%.0fg/leg): hip-pitch %.2f kg-cm (SG90 x%.1f)  knee %.2f (x%.1f)"
              % (total_g, legload*1000, tau_hip, STALL_SG90/tau_hip, tau_knee, STALL_SG90/tau_knee))
print("\nStatic hold; walking/dynamic ~2-3x. If any margin <2x, drop to MG90S (2.2, same footprint).")
