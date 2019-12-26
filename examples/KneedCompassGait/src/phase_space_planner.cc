#include "drake/examples/KneedCompassGait/include/phase_space_planner.h"


namespace phase_space_planner
{

  Eigen::Matrix<double, 2, 1> PhaseSpacePlanner::ForwardProp(
      double p_f, double p, double p_dot, 
      double h, double aq, double eps, double count) 
  {
    double w_sq;
    double p_ddot;
    double inc_p;
    Eigen::Matrix<double, 2, 1> result;

    for (int i = 0; i < count; i++)
    {
      w_sq = G / h;

      p_ddot = w_sq * (p - p_f);
      inc_p = eps * p_dot + 0.5 * eps * eps * p_ddot;
      p = p + inc_p;
      p_dot = p_dot + eps * p_ddot;

      h = h + aq * inc_p;
    }

    result(0, 0) = p;
    result(1, 0) = p_dot;

    return result;
  }

  void PhaseSpacePlanner::Init(Eigen::Matrix<double, 5, 1>& apex_init,
                               Eigen::Matrix<double, 5, 1>& d_init,
                               Eigen::Matrix<double, 3, 1>& p_foot_init)
  {
    X_apex = apex_init;
    X_d = d_init;
    p_foot = p_foot_init;

    apex_list.push_back(X_apex);
    d_list.push_back(X_d);
    p_foot_list.push_back(p_foot);

    std::cout << "System Initialized." << std::endl;

    std::cout << std::endl;
  }

  void PhaseSpacePlanner::UpdatePrimitive(Action& action) 
  {
    prim(0, 0) = action.step_length;
    prim(1, 0) = action.dheading;
    prim(2, 0) = action.dheight;
    prim(3, 0) = action.v_apex;
    prim(4, 0) = action.h_apex;

    // debug
    std::cout << "Primitive Updated." << std::endl;

    std::cout << std::endl;
    std::cout << "step_length:" << prim(0, 0) << std::endl;
    std::cout << "dheading:" << prim(1, 0) << std::endl;
    std::cout << "dheight:" << prim(2, 0) << std::endl;
    std::cout << "v_apex:" << prim(3, 0) << std::endl;
    std::cout << "h_apex:" << prim(4, 0) << std::endl;

    std::cout << std::endl;
  }

  void PhaseSpacePlanner::UpdateKeyframe() 
  {
    double COS = std::cos(X_apex(3, 0) + prim(1, 0));
    double SIN = std::sin(X_apex(3, 0) + prim(1, 0));

    // Keyframe1
    double s1 = (X_apex(0, 0) - X_d(0, 0))*COS + 
                (X_apex(1, 0) - X_d(1, 0))*SIN;
    double s1_dot = X_apex(4, 0)*std::cos(prim(1, 0));
    double s1_ddot;

    double l1 = -(X_apex(0, 0) - X_d(0, 0))*SIN + 
                 (X_apex(1, 0) - X_d(1, 0))*COS;
    double l1_dot = -X_apex(4, 0)*std::sin(prim(1, 0));
    double l1_ddot;

    double v1 = X_apex(2, 0);
    double v1_dot;
    double v1_ddot;

    // Foot1
    double s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + 
                     (p_foot(1, 0) - X_d(1, 0))*SIN;
    double l1_foot = -(p_foot(0, 0) - X_d(0, 0))*SIN + 
                      (p_foot(1, 0) - X_d(1, 0))*COS;
    double v1_foot = p_foot(2, 0);

    // Keyframe2
    double s2 = prim(0, 0);
    double s2_dot = prim(3, 0);
    double s2_ddot;
    
    double l2;
    double l2_dot = 0;
    double l2_ddot;
    
    double v2 = p_foot(2, 0) + prim(2, 0) + prim(4, 0);
    double v2_dot;
    double v2_ddot;

    // Foot2
    double s2_foot = prim(0, 0);
    double l2_foot;
    double v2_foot = p_foot(2, 0) + prim(2, 0);

    // Switch
    double s_switch;
    double ds_switch;

    double l_switch;
    double dl_switch;

    double v_switch;
    double dv_switch;


    // Forward-Backward Prop for Sagittal Switch
    double eps = 0.001;
    double forward_count = 0;
    double backward_count = 0;
    double w1_sq = 0;
    double w2_sq = 0;
    double aq = (v2 - v1) / (s2 - s1);
    double bq = 0;
    double inc_s1, inc_l1, inc_s2;
    
    while (s2 > s1) 
    {
      // Forward Prop
      if (s1_dot < s2_dot) 
      {
        w1_sq = G / (v1 - v1_foot);

        s1_ddot = w1_sq * (s1 - s1_foot);
        inc_s1 = eps * s1_dot + 0.5 * eps * eps * s1_ddot;
        s1 = s1 + inc_s1;
        s1_dot = s1_dot + eps * s1_ddot;

        l1_ddot = w1_sq * (l1 - l1_foot);
        inc_l1 = eps * l1_dot + 0.5 * eps * eps * l1_ddot;
        l1 = l1 + inc_l1;
        l1_dot = l1_dot + eps * l1_ddot;

        v1_ddot = aq * s1_ddot + bq * l1_ddot;
        v1 = v1 + aq * inc_s1 + bq * inc_l1;
        v1_dot = aq * s1_dot + bq * l1_dot;

        forward_count = forward_count + 1;
      } 
      // Backward Prop
      else 
      {
        w2_sq = G / (v2 - v2_foot);

        s2_ddot = w2_sq * (s2 - s2_foot);
        inc_s2 = - eps * s2_dot + 0.5 * eps * eps * s2_ddot;
        s2 = s2 + inc_s2;
        s2_dot = s2_dot - eps * s2_ddot;

        v2_ddot = aq * s2_ddot;
        v2 = v2 + aq * inc_s2;
        v2_dot = aq * s2_dot;

        backward_count = backward_count + 1;
      }
    }

    s_switch = (s1 + s2) / 2;
    ds_switch = (s1_dot + s2_dot) / 2;

    l_switch = l1;
    dl_switch = l1_dot;

    v_switch = (v1 + v2) / 2;
    dv_switch = (v1_dot + v2_dot) / 2;

    // Newton-Raphson Search for Lateral Foot Placement
    int n = 1, n_max = 50;
    double tol = 0.001;
    double pre_foot, pre_dot;
    Eigen::Matrix<double, 2, 1> res;

    if (stance == 1)
      l2_foot = -0.15;
    else
      l2_foot = 0.15;

    res = ForwardProp(l2_foot, l1, l1_dot, v_switch - v2_foot, aq, eps, backward_count);
    l2_dot = res(1, 0);
    l2_ddot = w2_sq * (res(0, 0) - l2_foot);
    
    while (n < n_max && std::abs(l2_dot) > tol)
    {
      pre_foot = l2_foot;
      l2_foot = l2_foot - l2_dot/l2_ddot;
      pre_dot = l2_dot;
      res = ForwardProp(l2_foot, l1, l1_dot, v_switch - v2_foot, aq, eps, backward_count);
      l2_dot = res(1, 0);
      l2_ddot = (l2_dot - pre_dot) / (l2_foot - pre_foot);
      n = n + 1;
    }

    s2 = prim(0, 0);
    s2_dot = prim(3, 0);

    l2 = res(0, 0);
    l2_dot = res(1, 0);

    v2 = p_foot(2, 0) + prim(2, 0) + prim(4, 0);
    v2_dot = aq * s2_dot;


    // p_foot
    p_foot(0, 0) = s2_foot*COS - l2_foot*SIN + X_d(0, 0);
    p_foot(1, 0) = s2_foot*SIN + l2_foot*COS + X_d(1, 0);
    p_foot(2, 0) = p_foot(2, 0) + prim(2, 0);
    if (stance == 0)
    {
      stance = 1;
    }
    else
    {
      stance = 0;
    }
    
    // X_switch
    X_switch(0, 0) = s_switch*COS - l_switch*SIN + X_d(0, 0);
    X_switch(1, 0) = s_switch*SIN + l_switch*COS + X_d(1, 0);
    X_switch(2, 0) = v_switch;
    X_switch(3, 0) = ds_switch*COS - dl_switch*SIN;
    X_switch(4, 0) = ds_switch*SIN + dl_switch*COS;
    X_switch(5, 0) = dv_switch;

    // Keyframe 
    X_apex(0, 0) = s2*COS - l2*SIN + X_d(0, 0);
    X_apex(1, 0) = s2*SIN + l2*COS + X_d(1, 0);
    X_apex(2, 0) = p_foot(2, 0) + prim(4, 0);
    X_apex(3, 0) += prim(1, 0);
    X_apex(4, 0) = prim(3, 0);

    X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0));
    X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0));
    X_d(2, 0) = p_foot(2, 0) + prim(4, 0);
    X_d(3, 0) += prim(1, 0);
    X_d(4, 0) = prim(3, 0);


    // Record
    step += 1;
    apex_list.push_back(X_apex);
    d_list.push_back(X_d);
    switch_list.push_back(X_switch);
    p_foot_list.push_back(p_foot);
    Eigen::Matrix<double, 2, 1> dt;
    dt << eps*forward_count, eps*backward_count;
    step_period.push_back(dt);
    Eigen::Matrix<double, 2, 1> slope;
    slope << aq * std::cos(X_apex(3, 0)), aq * std::sin(X_apex(3, 0));
    step_surface.push_back(slope);


    // debug
    std::cout << "Keyframe Updated." << std::endl;

    std::cout << std::endl;
    std::cout << "s_apex:" << s2 << std::endl;
    std::cout << "sdot_apex:" << s2_dot << std::endl;
    std::cout << "l_apex:" << l2 << std::endl;
    std::cout << "ldot_apex:" << l2_dot << std::endl;
    std::cout << "v_apex:" << v2 << std::endl;
    std::cout << "vdot_apex:" << v2_dot << std::endl;

    std::cout << std::endl;
    std::cout << "x_apex:" << X_apex(0, 0) << std::endl;
    std::cout << "y_apex:" << X_apex(1, 0) << std::endl;
    std::cout << "z_apex:" << X_apex(2, 0) << std::endl;
    std::cout << "theta_apex:" << X_apex(3, 0) << std::endl;
    std::cout << "v_apex:" << X_apex(4, 0) << std::endl;

    std::cout << std::endl;
    std::cout << "x_d:" << X_d(0, 0) << std::endl;
    std::cout << "y_d:" << X_d(1, 0) << std::endl;
    std::cout << "z_d:" << X_d(2, 0) << std::endl;
    std::cout << "theta_d:" << X_d(3, 0) << std::endl;
    std::cout << "v_d:" << X_d(4, 0) << std::endl;

    std::cout << std::endl;
    std::cout << "x_switch:" << X_switch(0, 0) << std::endl;
    std::cout << "y_switch:" << X_switch(1, 0) << std::endl;
    std::cout << "z_switch:" << X_switch(2, 0) << std::endl;
    std::cout << "dx_switch:" << X_switch(3, 0) << std::endl;
    std::cout << "dy_switch:" << X_switch(4, 0) << std::endl;
    std::cout << "dz_switch:" << X_switch(5, 0) << std::endl;

    std::cout << std::endl;
    std::cout << "x_foot:" << p_foot(0, 0) << std::endl;
    std::cout << "y_foot:" << p_foot(1, 0) << std::endl;
    std::cout << "z_foot:" << p_foot(2, 0) << std::endl;

    std::cout << std::endl;
  }

  void PhaseSpacePlanner::CalOneStepTraj() 
  {
    double T = step_period[step-2](1, 0) + step_period[step-1](0, 0);
    double dt = 0.001;
    int num = T/dt;

    Eigen::Matrix<double, 6, 1> start = switch_list[step-2];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];
    Eigen::Matrix<double, 2, 1> slope1 = step_surface[step-2];
    Eigen::Matrix<double, 2, 1> slope2 = step_surface[step-1];
    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-2];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot = p_foot_list[step];

    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);

    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;

    double x_distance = nex_foot(0, 0) - pre_foot(0, 0); 
    double y_distance = nex_foot(1, 0) - pre_foot(1, 0);
    double dh = 0.3;
    double h1 = dh;
    double h2 = dh + pre_foot(2, 0) - nex_foot(2, 0);
    double t1, t2, ts;
    double vmax = 0.5;
    bool steady = false;

    if (h1 > h2)
    {
      t1 = h1 * PI / (2 * vmax);
      t2 = t1 * h2 / h1;
    }
    else
    {
      t2 = h2 * PI / (2 * vmax);
      t1 = t2 * h1 / h2;
    }
    
    if (T < t1 + t2)
    {
      h1 = (T * h1 / (h1 + h2)) * 2 * vmax / PI;
      h2 = h1 + pre_foot(2, 0) - nex_foot(2, 0);
      t1 = T * h1 / (h1 + h2);
      t2 = T * h2 / (h1 + h2);
      ts = 0;
    }
    else
    {
      steady = true;
      ts = T - t1 - t2;
    }

    int num0 = step_period[step-2](1, 0) / dt;
    int num1 = t1 / dt;
    int num2 = t2 / dt;
    int num3 = ts / dt;

    double w_sq;
    double inc_x, inc_y;

    for (int i = 0; i < num; i++)
    {
      w_sq = G / (z - cur_foot(2, 0)) ;

      x_ddot = w_sq * (x - cur_foot(0, 0));
      inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
      x = x + inc_x;
      x_dot = x_dot + dt * x_ddot;

      y_ddot = w_sq * (y - cur_foot(1, 0));
      inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
      y = y + inc_y;
      y_dot = y_dot + dt * y_ddot;

      if (i < num0)
      {
        z_ddot = slope1(0, 0) * x_ddot + slope1(1, 0) * y_ddot;
        z = z + slope1(0, 0) * inc_x + slope1(1, 0) * inc_y;
        z_dot = slope1(0, 0) * x_dot + slope1(1, 0) * y_dot;
      }
      else
      {
        z_ddot = slope2(0, 0) * x_ddot + slope2(1, 0) * y_ddot;
        z = z + slope2(0, 0) * inc_x + slope2(1, 0) * inc_y;
        z_dot = slope2(0, 0) * x_dot + slope2(1, 0) * y_dot;
      }
      

      Eigen::Matrix<double, 9, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot;
      COM_list.push_back(COM);

      x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      if(i < num1)
      {
        z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
        dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
        ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      }
      else if (i < num1 + num3)
      {
        z_swing = pre_foot(2, 0) + h1;
        dz_swing = 0;
        ddz_swing = 0;
      }
      else
      {
        z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 9, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing;
      Eigen::Matrix<double, 9, 1> stance_foot;
      stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0;

      if (stance == 0)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
    }
  }

}  // namespace phase_space_planner