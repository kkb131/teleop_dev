#!/usr/bin/env python3
"""Step 1: OpenVR connection test.

Verifies that SteamVR is running and can be initialized.
Lists all connected tracker devices.

Requirements: SteamVR running, openvr installed
Usage: python3 -m vive.tests.test_step1_openvr
"""

import sys


def main():
    print("=" * 50)
    print("  Step 1: OpenVR Connection Test")
    print("=" * 50)
    passed = 0
    failed = 0

    # Test 1: Import openvr
    print("\n[TEST] Import openvr...", end=" ")
    try:
        import openvr
        print("[PASS]")
        passed += 1
    except ImportError:
        print("[FAIL] openvr not installed. Run: pip install openvr")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Initialize OpenVR
    print("[TEST] Initialize OpenVR (VRApplication_Other)...", end=" ")
    try:
        vr_system = openvr.init(openvr.VRApplication_Other)
        print("[PASS]")
        passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        print("       Is SteamVR running? Check null driver config.")
        failed += 1
        _summary(passed, failed)
        return

    # Test 3: Enumerate devices
    print("[TEST] Enumerate tracked devices...", end=" ")
    try:
        trackers = []
        controllers = []
        base_stations = []
        hmds = []

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            dev_class = vr_system.getTrackedDeviceClass(i)
            if dev_class == openvr.TrackedDeviceClass_Invalid:
                continue

            serial = vr_system.getStringTrackedDeviceProperty(
                i, openvr.Prop_SerialNumber_String
            )

            if dev_class == openvr.TrackedDeviceClass_GenericTracker:
                trackers.append({"index": i, "serial": serial})
            elif dev_class == openvr.TrackedDeviceClass_Controller:
                controllers.append({"index": i, "serial": serial})
            elif dev_class == openvr.TrackedDeviceClass_TrackingReference:
                base_stations.append({"index": i, "serial": serial})
            elif dev_class == openvr.TrackedDeviceClass_HMD:
                hmds.append({"index": i, "serial": serial})

        print("[PASS]")
        passed += 1

        print(f"\n  Devices found:")
        print(f"    HMDs:          {len(hmds)}")
        print(f"    Controllers:   {len(controllers)}")
        print(f"    Base Stations: {len(base_stations)}")
        print(f"    Trackers:      {len(trackers)}")

        for bs in base_stations:
            print(f"      Base Station: index={bs['index']}  serial={bs['serial']}")
        for t in trackers:
            print(f"      Tracker:      index={t['index']}  serial={t['serial']}")

    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # Test 4: Check tracker tracking state
    if trackers:
        print(f"\n[TEST] Check tracker tracking state...", end=" ")
        try:
            poses = vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0.0,
                openvr.k_unMaxTrackedDeviceCount,
            )
            tracking_ok = 0
            for t in trackers:
                pose = poses[t["index"]]
                status = "TRACKING" if pose.bPoseIsValid else "NOT TRACKING"
                print(f"\n    {t['serial']}: [{status}]", end="")
                if pose.bPoseIsValid:
                    tracking_ok += 1

            if tracking_ok > 0:
                print(f"\n  [PASS] {tracking_ok}/{len(trackers)} tracker(s) actively tracking")
                passed += 1
            else:
                print(f"\n  [WARN] No trackers actively tracking")
                print("         Check base station visibility and tracker power")
                failed += 1
        except Exception as e:
            print(f"[FAIL] {e}")
            failed += 1
    else:
        print("\n[WARN] No trackers found — cannot test tracking state")
        print("       Pair your Vive Tracker via SteamVR → Devices → Pair Controller")

    # Cleanup
    openvr.shutdown()

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 1 complete — proceed to Step 2")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 50}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
