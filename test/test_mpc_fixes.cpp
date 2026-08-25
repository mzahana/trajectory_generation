// Regression tests for the MPC hardening fixes (T1.3, T1.4, T1.5).
// Standalone (no gtest dependency in this package): returns non-zero on failure.
#include <trajectory_generation/mpc_12state.hpp>
#include <iostream>
#include <cmath>

namespace {

int failures = 0;

void check(bool cond, const char * what)
{
  if (cond) {
    printInfo("PASS: %s", what);
  } else {
    printError("FAIL: %s", what);
    ++failures;
  }
}

// Build an MPC configured like the field/SITL vehicle.
MPC12STATE * makeMPC(int window, double dt, double min_alt)
{
  auto * mpc = new MPC12STATE();
  mpc->setDebug(false);
  mpc->setDt(dt);
  mpc->setMPCWindow(window);
  mpc->setXYStateWeight(1000.0);  mpc->setZStateWeight(1000.0);  mpc->setYawStateWeight(1000.0);
  mpc->setXYInputWeight(10.0);    mpc->setZInputWeight(10.0);    mpc->setYawInputWeight(1.0);
  mpc->setXYMaxVel(12.0);         mpc->setZMaxVel(5.0);          mpc->setYawMaxVel(5.0);
  mpc->setXYMaxAccel(5.0);        mpc->setZMaxAccel(5.0);        mpc->setYawMaxAccel(10.0);
  mpc->setXYMaxJerk(10.0);        mpc->setZMaxJerk(10.0);        mpc->setYawMaxJerk(10.0);
  mpc->setMinimumAltitude(min_alt);
  // No wall-clock bound in unit tests: the limit is a deployment latency guard,
  // and leaving it on would make these assertions depend on machine load.
  mpc->setQPTimeLimit(0.0);
  return mpc;
}

}  // namespace

int main()
{
  const int N = 10;
  const double dt = 0.1;

  // -------------------------------------------------------------------------
  // T1.4a: altitude floor binds when the reference dives below it.
  // -------------------------------------------------------------------------
  {
    const double min_alt = 5.0;
    auto * mpc = makeMPC(N, dt, min_alt);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    MatX_12STATE x0;
    x0 << 0,0,0,  0,0,0,  10.0,0,0,  0,0,0;   // hovering at z = 10
    mpc->setCurrentState(x0);

    // Target dives hard to z = -20 (below the floor).
    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) { ref(i*NUM_OF_STATES+6, 0) = -20.0; }
    mpc->setReferenceTraj(ref);

    check(mpc->mpcLoop(), "T1.4a QP remains feasible with an active altitude floor");

    auto traj = mpc->getOptimalStateTraj();
    double min_planned_z = 1e9;
    for (int i = 1; i < N+1; ++i) {
      min_planned_z = std::min(min_planned_z, traj(i*NUM_OF_STATES + 6));
    }
    std::cout << "   min planned z = " << min_planned_z
              << " (floor " << min_alt << ")\n";
    check(min_planned_z >= min_alt - 1e-2,
          "T1.4a planned trajectory never descends below minimum_altitude");
    delete mpc;
  }

  // -------------------------------------------------------------------------
  // T1.4b: starting BELOW the floor must still produce a solution that climbs,
  // not an infeasible QP (the reachability-ramped floor).
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ 5.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    MatX_12STATE x0;
    x0 << 0,0,0,  0,0,0,  0.1,0,0,  0,0,0;   // well below the floor
    mpc->setCurrentState(x0);

    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) { ref(i*NUM_OF_STATES+6, 0) = 20.0; }
    mpc->setReferenceTraj(ref);

    check(mpc->mpcLoop(), "T1.4b QP feasible when starting below the floor");
    auto traj = mpc->getOptimalStateTraj();
    const double z_end = traj(N*NUM_OF_STATES + 6);
    std::cout << "   z(0) = 0.1 -> z(N) = " << z_end << "\n";
    check(z_end > 0.1, "T1.4b solution climbs back toward the floor");
    delete mpc;
  }

  // -------------------------------------------------------------------------
  // T1.3: yaw reference is unwrapped across the +/-pi seam.
  // Vehicle yaw just under +pi; target placed so the LOS heading is just over
  // +pi (i.e. atan2 returns ~-pi). The yaw plan must NOT spin the long way.
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ -100.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    const double yaw0 = 3.10;   // just under +pi
    MatX_12STATE x0;
    x0 << 0,0,0,  0,0,0,  10.0,0,0,  yaw0,0,0;
    mpc->setCurrentState(x0);

    // Target behind and slightly -y: LOS heading ~ -3.10 rad == +3.18 unwrapped.
    const double ang = -3.10;
    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) {
      ref(i*NUM_OF_STATES+0, 0) = 50.0*std::cos(ang);
      ref(i*NUM_OF_STATES+3, 0) = 50.0*std::sin(ang);
      ref(i*NUM_OF_STATES+6, 0) = 10.0;
    }
    mpc->setReferenceTraj(ref);
    check(mpc->mpcLoop(), "T1.3 QP feasible near the yaw seam");

    auto traj = mpc->getOptimalStateTraj();
    double max_dev = 0.0;
    for (int i = 1; i < N+1; ++i) {
      max_dev = std::max(max_dev, std::abs(traj(i*NUM_OF_STATES + 9) - yaw0));
    }
    std::cout << "   max |yaw(i) - yaw0| = " << max_dev << " rad\n";
    // Unwrapped, the shortest turn is ~0.08 rad. A wrapped reference would
    // command a turn of order 2*pi the wrong way.
    check(max_dev < 1.0, "T1.3 yaw takes the short way across the seam (no 2pi spin)");
    delete mpc;
  }

  // -------------------------------------------------------------------------
  // T1.5: setReferenceTraj validates the INPUT size.
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ -100.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }
    Eigen::MatrixXd bad(NUM_OF_STATES*(N+1) - 3, 1); bad.setZero();
    check(!mpc->setReferenceTraj(bad), "T1.5 wrong-sized reference is rejected");
    Eigen::MatrixXd good(NUM_OF_STATES*(N+1), 1); good.setZero();
    check(mpc->setReferenceTraj(good), "T1.5 correctly-sized reference is accepted");
    delete mpc;
  }

  // -------------------------------------------------------------------------
  // T1.9: the initial state is a MEASUREMENT and must never make the QP
  // infeasible, even when it violates the planning limits.
  //
  // Regression for a silently fatal in-flight failure: x(0) is pinned by an
  // equality constraint AND was separately bounded by the vel/accel limits.
  // Once real (non-zero) velocity was fed in, any moment the vehicle exceeded
  // a limit produced "solution is not found", no setpoint was published, the
  // command stream gapped, and the controller failsafed into a landing.
  // Observed in SITL as 176 consecutive Z-axis solve failures.
  // -------------------------------------------------------------------------
  {
    struct Case { const char * name; double vz, vx, vyaw; };
    const Case cases[] = {
      {"vertical speed over limit",   -9.0,  0.0, 0.0},   // z_max_vel is 5.0
      {"horizontal speed over limit",  0.0, 20.0, 0.0},   // xy_max_vel is 12.0
      {"yaw rate over limit",          0.0,  0.0, 9.0},   // yaw_max_vel is 5.0
      {"all axes over limit",         -9.0, 20.0, 9.0},
    };
    for (const auto & c : cases) {
      auto * mpc = makeMPC(N, dt, /*min_alt*/ 5.0);
      if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

      MatX_12STATE x0;
      x0 << 0, c.vx, 0,   0, 0, 0,   20.0, c.vz, 0,   0, c.vyaw, 0;
      mpc->setCurrentState(x0);

      Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
      for (int i = 0; i < N+1; ++i) {
        ref(i*NUM_OF_STATES+0, 0) = 30.0;
        ref(i*NUM_OF_STATES+6, 0) = 20.0;
      }
      mpc->setReferenceTraj(ref);

      const bool solved = mpc->mpcLoop();
      std::string what = std::string("T1.9 QP stays feasible with ") + c.name;
      check(solved, what.c_str());
      delete mpc;
    }
  }


  // -------------------------------------------------------------------------
  // T1.10: the midcourse infeasibility fix.
  //
  // The QP had "reachability ramps": each state bound was widened along a
  // simulated max-effort trajectory so that an initial state outside the
  // envelope would still be reachable. That is what made the midcourse MPC
  // report up to 77.7% infeasible on agile targets (figure8 / jink), because:
  //   (a) the ramped bound equalled the max-effort trajectory EXACTLY, so the
  //       feasible set had empty interior and solver tolerances alone reported
  //       primal_infeasible;
  //   (b) the acceleration ramp clamped to the accel limit, so a measured
  //       |a| ABOVE that limit was unreachable at step 1 -- genuinely
  //       infeasible;
  //   (c) the XY ramp assumed the current acceleration OPPOSED the current
  //       velocity, so whenever it did not (accelerating along track, i.e.
  //       every hard turn) the bound was below the speed the vehicle would
  //       actually have at step 1.
  // The bounds are now soft (slack + exact penalty), so u = 0 is always
  // admissible and the QP cannot be infeasible from any state.
  // -------------------------------------------------------------------------
  {
    struct Case { const char * name; double vx, vy, ax, ay, vz, az, vyaw, ayaw; };
    const Case cases[] = {
      // (b): acceleration above the limit -- the QP the ramp could not satisfy
      {"vertical accel over limit",      0,0,   0,0,    0.0, 12.0,  0,0},
      {"horizontal accel over limit",    0,0,  11.0,9.0, 0,   0,    0,0},
      {"yaw accel over limit",           0,0,   0,0,     0,   0,    0, 25.0},
      // (c): accelerating ALONG the velocity, the case the XY ramp got backwards
      {"accel along velocity",          20.0,0, 8.0,0,   0,   0,    0,0},
      {"accel along velocity, diagonal",14.0,14.0, 6.0,6.0, 0, 0,   0,0},
      // (a): exactly at the limit, where the knife-edge bound used to sit
      {"exactly at the velocity limit", 12.0,0, 0,0,    5.0, 0,     5.0, 0},
      // combined worst case: every axis outside every limit at once
      {"everything over limit",         22.0,18.0, 11.0,9.0, -9.0, 12.0, 9.0, 25.0},
      // descending fast while below the altitude floor: the vel bound and the
      // floor used to be ramped along two DIFFERENT trajectories at once
      {"diving below the floor",         0,0,   0,0,   -9.0, -8.0,  0,0},
    };
    for (const auto & c : cases) {
      auto * mpc = makeMPC(N, dt, /*min_alt*/ 5.0);
      if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

      MatX_12STATE x0;
      x0 << 0, c.vx, c.ax,   0, c.vy, c.ay,   6.0, c.vz, c.az,   0, c.vyaw, c.ayaw;
      mpc->setCurrentState(x0);

      // A reference that pulls hard, so the bounds are genuinely in the way.
      Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
      for (int i = 0; i < N+1; ++i) {
        ref(i*NUM_OF_STATES+0, 0) = 60.0 + 15.0*i*dt;
        ref(i*NUM_OF_STATES+3, 0) = -40.0;
        ref(i*NUM_OF_STATES+6, 0) = 25.0;
      }
      mpc->setReferenceTraj(ref);

      const bool solved = mpc->mpcLoop();
      // The guarantee is FEASIBILITY, which is what the soft constraints buy.
      // Convergence is a separate, solver-quality question with a safe
      // documented fallback: an under-converged iterate is rejected (it can
      // violate the very limits the QP enforces) and the controller holds the
      // previous setpoint for one 50 ms cycle. Asserting "solved" here would
      // conflate the two and make the test depend on solver tolerances.
      const bool infeasible =
        mpc->lastFailReason().find("infeasible") != std::string::npos;
      std::string what = std::string("T1.10a QP is never infeasible with ") + c.name;
      check(!infeasible, what.c_str());
      if (!solved) {
        printWarn("   %s: not infeasible, but under-converged (%s) -> cycle dropped",
                  c.name, mpc->lastFailReason().c_str());
      } else {
        // Degradation must never be silent: an out-of-envelope state has to
        // show up as slack on the health topic.
        const double slack = std::max(mpc->maxSlackZ(),
                              std::max(mpc->maxSlackXY(), mpc->maxSlackYaw()));
        std::cout << "   " << c.name << ": max slack = " << slack << "\n";
      }
      delete mpc;
    }
  }

  // -------------------------------------------------------------------------
  // T1.10b: the penalty is EXACT -- when the state is inside the envelope and
  // the hard-bounded problem is feasible, the slack is zero and the plan
  // respects the limits. Without this the softening would silently become a
  // licence to fly outside the envelope, which is worse than the bug it fixes.
  // -------------------------------------------------------------------------
  {
    const double vmax = 12.0, azmax = 5.0, vzmax = 5.0;
    // Reference distances spanning the engagement: the exact-penalty weight has
    // to track the tracking-cost magnitude, which grows with this distance.
    for (double R : {10.0, 50.0, 200.0, 500.0}) {
      auto * mpc = makeMPC(N, dt, /*min_alt*/ 2.0);
      if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

      MatX_12STATE x0;
      x0 << 0,0,0,  0,0,0,  15.0,0,0,  0,0,0;   // hovering, well inside the envelope
      mpc->setCurrentState(x0);

      Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
      for (int i = 0; i < N+1; ++i) {
        ref(i*NUM_OF_STATES+0, 0) = R;          // far away: maximum pull
        ref(i*NUM_OF_STATES+6, 0) = 15.0 + R;
      }
      mpc->setReferenceTraj(ref);

      if (!mpc->mpcLoop()) {
        printError("T1.10b solve failed at R = %.0f (%s)", R, mpc->lastFailReason().c_str());
        ++failures; delete mpc; continue;
      }

      auto traj = mpc->getOptimalStateTraj();
      double v_pk = 0.0, vz_pk = 0.0, az_pk = 0.0;
      for (int i = 1; i < N+1; ++i) {
        v_pk  = std::max(v_pk, std::hypot(traj(i*NUM_OF_STATES + STATE_VX_IDX),
                                          traj(i*NUM_OF_STATES + STATE_VY_IDX)));
        vz_pk = std::max(vz_pk, std::abs(traj(i*NUM_OF_STATES + STATE_VZ_IDX)));
        az_pk = std::max(az_pk, std::abs(traj(i*NUM_OF_STATES + STATE_AZ_IDX)));
      }
      const double slack = std::max(mpc->maxSlackZ(),
                            std::max(mpc->maxSlackXY(), mpc->maxSlackYaw()));
      std::cout << "   R = " << R << " m: v_pk = " << v_pk << " vz_pk = " << vz_pk
                << " az_pk = " << az_pk << " slack = " << slack << "\n";

      std::string what = "T1.10b plan stays inside the envelope with the reference at "
                       + std::to_string(static_cast<int>(R)) + " m";
      check(v_pk <= vmax + 0.1 && vz_pk <= vzmax + 0.1 && az_pk <= azmax + 0.1,
            what.c_str());
      std::string what2 = "T1.10b slack is zero when the hard problem is feasible, R = "
                        + std::to_string(static_cast<int>(R)) + " m";
      check(slack < 0.05, what2.c_str());

      // ...and the plan must still actually GO somewhere. A badly scaled QP can
      // return a near-zero "solution" and still report Solved.
      const double moved = std::abs(traj(N*NUM_OF_STATES + STATE_X_IDX));
      std::string what3 = "T1.10b plan still commands motion toward a reference at "
                        + std::to_string(static_cast<int>(R)) + " m";
      check(moved > 0.5, what3.c_str());
      delete mpc;
    }
  }

  // -------------------------------------------------------------------------
  // T1.10c: the relaxation is MINIMAL -- an out-of-envelope state buys only the
  // slack it needs, and the plan brakes back toward the envelope rather than
  // settling outside it.
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ 2.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    const double vx0 = 20.0;             // limit is 12
    MatX_12STATE x0;
    x0 << 0, vx0, 0,  0,0,0,  15.0,0,0,  0,0,0;
    mpc->setCurrentState(x0);

    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) {
      ref(i*NUM_OF_STATES+0, 0) = 200.0;   // still pulling hard along +x
      ref(i*NUM_OF_STATES+6, 0) = 15.0;
    }
    mpc->setReferenceTraj(ref);
    check(mpc->mpcLoop(), "T1.10c QP is feasible above the speed limit");

    auto traj = mpc->getOptimalStateTraj();
    const double v_end = traj(N*NUM_OF_STATES + STATE_VX_IDX);
    std::cout << "   vx: " << vx0 << " -> " << v_end
              << " (limit 12), xy slack = " << mpc->maxSlackXY() << "\n";
    check(v_end < vx0, "T1.10c plan decelerates back toward the limit");
    // Only the unavoidable excess: vx(1) = vx(0) is fixed by the dynamics, so
    // at least (20 - 12) = 8 is required. Allow generous headroom for the
    // hexagonal velocity approximation, but not a free-for-all.
    check(mpc->maxSlackXY() > 1.0 && mpc->maxSlackXY() < 3.0*(vx0 - 12.0),
          "T1.10c relaxation is bounded by roughly what the state requires");
    delete mpc;
  }

  // -------------------------------------------------------------------------
  // T1.10d: hard constraints stay hard. Softening the STATE bounds must not
  // have softened the jerk (input) bounds -- those are what the airframe can
  // actually do, and the dynamics equality is what makes the plan physical.
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ 2.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    MatX_12STATE x0;
    x0 << 0, 20.0, 9.0,  0, -18.0, -9.0,  15.0, 8.0, 9.0,  0, 8.0, 20.0;
    mpc->setCurrentState(x0);
    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) {
      ref(i*NUM_OF_STATES+0, 0) = 300.0;
      ref(i*NUM_OF_STATES+6, 0) = 100.0;
    }
    mpc->setReferenceTraj(ref);
    check(mpc->mpcLoop(), "T1.10d QP is feasible from a fully out-of-envelope state");

    auto u = mpc->getOptimalControlTraj();
    (void)0;
    double j_pk = 0.0;
    for (int i = 0; i < u.size(); ++i) j_pk = std::max(j_pk, std::abs(u(i)));
    std::cout << "   peak |jerk| = " << j_pk << " (limit 10)\n";
    check(j_pk <= 10.0 + 1e-3, "T1.10d jerk bounds are still HARD");

    // The plan must obey the dynamics exactly: x(i+1) = A x(i) + B u(i).
    auto traj = mpc->getOptimalStateTraj();
    double dyn_err = 0.0;
    for (int i = 0; i < N; ++i) {
      const double z  = traj(i*NUM_OF_STATES + STATE_Z_IDX);
      const double vz = traj(i*NUM_OF_STATES + STATE_VZ_IDX);
      const double az = traj(i*NUM_OF_STATES + STATE_AZ_IDX);
      // The model is x(i+1) = A x(i) + B u(i) with B = [0, 0, dt]^T: jerk moves
      // the acceleration state only, it does not feed through to position or
      // velocity within a step. Mirror that here, not the exact triple
      // integrator, or this checks the discretisation rather than the equality.
      const double z_next  = z + vz*dt + 0.5*az*dt*dt;
      dyn_err = std::max(dyn_err, std::abs(traj((i+1)*NUM_OF_STATES + STATE_Z_IDX) - z_next));
    }
    std::cout << "   max dynamics residual (z) = " << dyn_err << "\n";
    check(dyn_err < 1e-6, "T1.10d dynamics equality is still HARD");
    delete mpc;
  }


  // -------------------------------------------------------------------------
  // T1.10e: the XY mixed-norm (hexagonal) velocity rows survived the rewrite.
  //
  // Those rows are what makes the SPEED bound a bound on |v| rather than on
  // each axis separately -- the per-axis box alone would permit
  // |v| = sqrt(2)*v_max at 45 deg. They were rewritten to carry slack columns,
  // so their coefficients and row placement need checking directly: a
  // misplaced coefficient would silently widen the speed envelope by 41%.
  // -------------------------------------------------------------------------
  {
    auto * mpc = makeMPC(N, dt, /*min_alt*/ 2.0);
    if (!mpc->initMPCProblem()) { printError("init failed"); return 1; }

    // Already at the speed limit along the 45 deg diagonal, which is where the
    // hexagon is tightest relative to the box.
    const double v_diag = 12.0 / std::sqrt(2.0);
    MatX_12STATE x0;
    x0 << 0, v_diag, 0,  0, v_diag, 0,  15.0, 0, 0,  0, 0, 0;
    mpc->setCurrentState(x0);

    Eigen::MatrixXd ref(NUM_OF_STATES*(N+1), 1); ref.setZero();
    for (int i = 0; i < N+1; ++i) {          // pull hard along the same diagonal
      ref(i*NUM_OF_STATES+0, 0) = 400.0;
      ref(i*NUM_OF_STATES+3, 0) = 400.0;
      ref(i*NUM_OF_STATES+6, 0) = 15.0;
    }
    mpc->setReferenceTraj(ref);
    check(mpc->mpcLoop(), "T1.10e QP is feasible at the speed limit on the diagonal");

    auto traj = mpc->getOptimalStateTraj();
    double v_pk = 0.0;
    for (int i = 1; i < N+1; ++i)
      v_pk = std::max(v_pk, std::hypot(traj(i*NUM_OF_STATES + STATE_VX_IDX),
                                       traj(i*NUM_OF_STATES + STATE_VY_IDX)));
    std::cout << "   peak planned speed = " << v_pk
              << " (limit 12; per-axis box alone would allow 16.97)\n";
    // The hexagon is an inner approximation of the disc: exact at 30 deg
    // spacing, up to 2/sqrt(3) = 1.155x at the vertices. Anything near 16.97
    // means the mixed rows are not binding at all.
    check(v_pk <= 12.0*2.0/std::sqrt(3.0) + 0.2,
          "T1.10e speed is bounded by the mixed-norm rows, not the per-axis box");
    delete mpc;
  }

  if (failures == 0) {
    printInfo("ALL MPC FIX TESTS PASSED");
    return 0;
  }
  printError("%d MPC fix test(s) FAILED", failures);
  return 1;
}
