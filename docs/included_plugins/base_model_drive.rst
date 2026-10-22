.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


Base Model Drive
================
A differential drive that follows measured behaviour of a real base. It replaces ``DiffDrive`` in a model file and
takes its dynamics from a parameter file instead of the ``linear_dynamics`` and ``angular_dynamics`` blocks.

Per physics step, with the length of that step: a command delay (rounded to whole steps), first-order lags, rate
limits (separate linear acceleration and deceleration, angular acceleration, jerk), a yaw-rate scale, and speed
caps. The same rigid-body motion is set on every body of the model, and odometry is published after the physics
step, so its twist is the velocity of the body, as on a real robot (``DiffDrive`` reports the commanded twist).

The plugin refuses to load, with an error message, on a missing parameter file, an unknown ``schema_version``, a
value out of range, or a ``linear_dynamics`` or ``angular_dynamics`` block. It warns once when the step size
differs by more than 1 % from the step the file was calibrated for. Parameter files and the calibration tool are in
``base_model/``; see ``base_model/README.md``.

.. code-block:: yaml

  plugins:

      # required, specify BaseModelDrive type to load the plugin
    - type: BaseModelDrive

      # required, name of the plugin
      name: robot_drive

      # required, exactly one of params_name (a file of base_model/params, installed with flatland_plugins)
      # and params_file (a path)
      params_name: example

      # required, body of a model to set velocities and obtain odometry
      body: base_link

      # all other keys of DiffDrive are accepted with the same meaning: odom_frame_id, twist_sub, odom_pub,
      # ground_truth_pub, twist_pub, enable_*_pub, pub_rate, odom_*_noise, odom_*_covariance

      # optional, defaults to base_model/provenance, latched topic with the parameter file name and its provenance
      provenance_pub: base_model/provenance
