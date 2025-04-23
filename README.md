# uav-rotor-control


### Thrust matrix
Constants used in thrust matrix can be found in:

/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf

        <visual name="5010_motor_base_0">
          <pose>0.174 0.174 .032 0 0 -.45</pose>     pose nr 1
          <geometry>
            <mesh>
              <scale>1 1 1</scale>
              <uri>model://x500_base/meshes/5010Base.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <visual name="5010_motor_base_1">
          <pose>-0.174 0.174 .032 0 0 -.45</pose>    pose nr 2
          <geometry>
            <mesh>
              <scale>1 1 1</scale>
              <uri>model://x500_base/meshes/5010Base.dae</uri>
            </mesh>
          </geometry>
        </visual>

and in:
/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf

    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>                     ct
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>       cq
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
