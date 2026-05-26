# Save as puma560_kin.py and run with Python 3 (requires numpy)
import numpy as np

# ---- DH parameters (a, alpha, d) -- theta is variable
DH = np.array([
    [0.0,     np.pi/2, 0.0],     # link 1
    [0.4318,  0.0,     0.0],     # link 2
    [0.0203, -np.pi/2, 0.15005], # link 3
    [0.0,     np.pi/2, 0.4318],  # link 4
    [0.0,    -np.pi/2, 0.0],     # link 5
    [0.0,     0.0,     0.0]      # link 6 (tool)
])

def dh_transform(a, alpha, d, theta):
    ca = np.cos(alpha); sa = np.sin(alpha)
    ct = np.cos(theta); st = np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])

def fkine(theta_vec):
    """Forward kinematics: theta_vec length 6 -> 4x4 transform T0_6"""
    T = np.eye(4)
    for i in range(6):
        a, alpha, d = DH[i]
        T = T @ dh_transform(a, alpha, d, theta_vec[i])
    return T

# helpers
def clamp(x, lo=-1.0, hi=1.0):
    return np.minimum(hi, np.maximum(lo, x))

def normalize_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# Analytic IK for PUMA-like geometry (returns list of possible solutions)
def ikine_puma560(T06, tol=1e-8):
    """
    Input: T06 4x4 desired pose
    Output: list of theta vectors (6,) in radians
    """
    R06 = T06[:3,:3]
    p06 = T06[:3,3]

    # Using the DH above, joint 6 offset along z6 is zero (tool at wrist)
    # But link 4 has d4 = 0.4318 (offset to wrist). We compute wrist point pw by subtracting
    # the effect of link 6 along end-effector z (here d6=0 -> pw == p06).
    # For safety, we'll compute pw = p06 - d6 * R06[:,2], with d6 = DH[5,2]
    d6 = DH[5,2]
    pw = p06 - d6 * R06[:,2]

    solutions = []

    # ----- theta1 (two possible: +/-) -----
    px, py, pz = pw
    # θ1 = atan2(py, px) +/- atan2(k, something) for offsets; with these DH simple atan2
    # two principal solutions:
    theta1_options = [np.arctan2(py, px)]
    # also add alternative by adding/subtracting pi (will be handled by later branches)
    theta1_options.append(normalize_angle(theta1_options[0] + np.pi))

    # lengths for shoulder/elbow triangle
    # compute wrist position in frame 1 coordinates
    for th1 in theta1_options:
        # transform wrist to frame 1 (but easier: compute planar distance from joint 2)
        # joint1 at base, joint2 axis offset along a2 on x1, etc. We'll follow standard geometric procedure.

        # position of wrist relative to joint1
        r1 = np.array([px, py, pz])

        # rotate into frame 1 by -theta1 about z
        Rz1 = np.array([[np.cos(-th1), -np.sin(-th1), 0],
                        [np.sin(-th1),  np.cos(-th1), 0],
                        [0, 0, 1]])
        p1 = Rz1 @ r1  # wrist pos in frame 1

        # coordinates for planar triangle (x along a2, z vertical)
        x = p1[0] - DH[1,0]  # subtract a2 (link between joint2 and joint3) if needed
        z = p1[2]            # height

        # lengths
        a2 = DH[1,0]
        a3 = DH[2,0]
        d3 = DH[2,2]

        # effective distance from joint2 to wrist in the plane
        D = np.hypot(x, z)

        # Law of cosines for theta3 (between link2 and link3)
        # Note: depending on your frame assignment sign conventions, theta3 formula may change.
        # We'll compute cos(theta3') where theta3' is the angle at the elbow triangle:
        # Using links: L1 = a2, L2 = sqrt(a3^2 + d3^2) (forearm effective)
        L1 = a2
        L2 = np.hypot(a3, d3)

        cos_gamma = clamp((D**2 - L1**2 - L2**2) / (2*L1*L2))
        # if |cos|>1 -> unreachable
        if abs(cos_gamma) > 1 - 1e-9:
            # unreachable for this theta1
            continue

        # two elbow configurations
        gamma_options = [np.arccos(cos_gamma), -np.arccos(cos_gamma)]
        for gamma in gamma_options:
            # compute theta2 from triangle
            beta = np.arctan2(z, x)  # angle to wrist
            # angle between L1 and line to wrist
            phi = np.arctan2(L2*np.sin(gamma), L1 + L2*np.cos(gamma))
            theta2 = normalize_angle(beta - phi)

            # compute theta3: need to map gamma to DH theta3
            # For this DH set, approximate mapping:
            theta3 = normalize_angle(gamma - np.arctan2(a3, d3))

            # Now compute R03 using these theta1..theta3
            thetas_123 = np.array([th1, theta2, theta3])
            T = np.eye(4)
            for i in range(3):
                a, alpha, d = DH[i]
                T = T @ dh_transform(a, alpha, d, thetas_123[i])
            R03 = T[:3,:3]

            # Solve wrist: R36 = R03^T * R06
            R36 = R03.T @ R06

            # extract wrist angles (theta4, theta5, theta6) from R36
            # using Z-Y-Z or the sequence implied by joints (here joints 4,5,6 are revolute)
            # We'll use:
            # theta5 = atan2( sqrt(R36[0,2]^2 + R36[1,2]^2), R36[2,2] )
            # theta4 = atan2(R36[1,2], R36[0,2])
            # theta6 = atan2(R36[2,1], -R36[2,0])
            # note: watch for singular (theta5 ~= 0)
            sy = np.hypot(R36[0,2], R36[1,2])
            if sy < 1e-9:
                # singular: theta5 ~ 0, infinite solutions for theta4+theta6
                theta5 = 0 if abs(R36[2,2] - 1) < 1e-6 else np.pi
                theta4 = 0.0
                theta6 = np.arctan2(-R36[0,1], R36[0,0])
                sol = np.array([th1, theta2, theta3, theta4, theta5, theta6])
                solutions.append(np.real_if_close(sol))
            else:
                theta5 = np.arctan2(sy, R36[2,2])
                theta4 = np.arctan2(R36[1,2], R36[0,2])
                theta6 = np.arctan2(R36[2,1], -R36[2,0])
                sol = np.array([th1, theta2, theta3, theta4, theta5, theta6])
                # also consider the alternate wrist (theta5 -> -theta5 and add pi shifts)
                sol2 = np.array([th1, theta2, theta3,
                                 normalize_angle(theta4 + np.pi),
                                 normalize_angle(-theta5),
                                 normalize_angle(theta6 + np.pi)])
                solutions.append(np.real_if_close(sol))
                solutions.append(np.real_if_close(sol2))

    # normalize angles and deduplicate (within tolerance)
    norm_sols = []
    for s in solutions:
        s = np.array([normalize_angle(x) for x in s])
        # de-dup
        dup = False
        for t in norm_sols:
            if np.allclose(s, t, atol=1e-6):
                dup = True; break
        if not dup:
            norm_sols.append(s)
    return norm_sols

# Example usage:
if __name__ == "__main__":
    # test: zero joint angles
    thetas_zero = np.zeros(6)
    T0 = fkine(thetas_zero)
    print("FK @ zeros:\n", T0)

    # IK: try to recover zeros from the computed pose
    sols = ikine_puma560(T0)
    print(f"Found {len(sols)} IK solutions.")
    for i,s in enumerate(sols):
        print(i, np.round(s,6))
