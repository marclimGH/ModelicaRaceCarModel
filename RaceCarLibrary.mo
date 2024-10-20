package RaceCarLibrary
import SI = Modelica.Units.SI;


   package DriversP
  partial class DriverInputs
      Real throttle(start = 0, min = 0, max = 1) "Throttle 0 to 1";
      Real brake(start = 0, min = 0, max = 1) "Brake 0 to 1";
      Modelica.Units.SI.Angle steeringAngle(start = 0, min = -Modelica.Constants.pi, max = Modelica.Constants.pi, displayUnit = "rad") "Steering Angle in radians";
  end DriverInputs;

    model SimpleDriver
      parameter Modelica.Units.SI.Time duration = 5 "time before reaching full throttle (s)";
      extends RaceCarLibrary.DriversP.DriverInputs;
    equation
      throttle = if time < duration then time/duration else 1;
      brake = 0;
      steeringAngle = 0;
    end SimpleDriver;
  end DriversP;

  package VehicleDynamicsP
    partial class baseLongitudinalDynamics
      parameter Modelica.Units.SI.Mass carMass = 800 "car mass in kg";
      outer Modelica.Units.SI.Force Fx "Net Longitudinal Force";
      outer Modelica.Units.SI.Force Ftraction "Traction Force";
      outer Modelica.Units.SI.Force Fbrake "Brake Force";
      outer Modelica.Units.SI.Force Fdrag "Drag Force";
      //  Modelica.Units.SI.Force Fslope "Slope Related Force (positive in the forward direction)";
      outer Modelica.Units.SI.Acceleration Ax "Longitudinal Acceleration";
      outer Modelica.Units.SI.Velocity Vx "Longitudinal Velocity";
 
    equation
      Fx = Ftraction - Fbrake - Fdrag;
      Fx = carMass*Ax;
      der(Vx) = Ax;
    end baseLongitudinalDynamics;

    model SimpleLongitudinalDynamics
      extends RaceCarLibrary.VehicleDynamicsP.baseLongitudinalDynamics;
    equation
    
    end SimpleLongitudinalDynamics;
  end VehicleDynamicsP;

  package TyreDynamicsP
  end TyreDynamicsP;

  package AeroDynamicsP
    partial class baseAerodynamics
      outer Modelica.Units.SI.Force Fdrag "Aeroudynamical Drag force";
    //  outer Modelica.Units.SI.Force Flift "Aeroudynamical Lift force";
      outer Modelica.Units.SI.Velocity Vx "Longitudinal Velocity";
    equation

    end baseAerodynamics;

    model SimpleAeroDynamics
     extends RaceCarLibrary.AeroDynamicsP.baseAerodynamics;
     parameter Real Cdrag(unit="kg/m")  "lumped drag coefficient (kg/m)";
    equation
    // Flift = 0;
     Fdrag = 0.5*Cdrag*(Vx^2);
    end SimpleAeroDynamics;
  end AeroDynamicsP;

  package EnvironmentsP
  end EnvironmentsP;

  package UtilitiesP
    record additionalTypes
    end additionalTypes;

    record sharedParameters
      
    end sharedParameters;
  end UtilitiesP;

  package SimulationsP 
    class baseMainSimulation
      parameter Modelica.Units.SI.Mass carMass "car mass in kg";
      parameter Real Cdrag;
      inner Modelica.Units.SI.Force Fx "Net Longitudinal Force";
      inner Modelica.Units.SI.Force Ftraction "Traction Force";
      inner Modelica.Units.SI.Force Fbrake "Brake Force";
      inner Modelica.Units.SI.Force Fdrag "Drag Force";
    //  inner Modelica.Units.SI.Force Flift "Lift Force";
      //  Modelica.Units.SI.Force Fslope "Slope Related Force (positive in the forward direction)";
      inner Modelica.Units.SI.Acceleration Ax(start = 0) "Longitudinal Acceleration";
      inner Modelica.Units.SI.Velocity Vx(start = 0)"Longitudinal Velocity";
      
      VehicleDynamicsP.SimpleLongitudinalDynamics LongitudinalDynamics(carMass = carMass)  annotation(
        Placement(transformation(origin = {14, -10}, extent = {{-44, -44}, {44, 44}})));
      DriversP.SimpleDriver Driver annotation(
        Placement(transformation(origin = {-74, -6}, extent = {{-10, -10}, {10, 10}})));
      AeroDynamicsP.SimpleAeroDynamics AeroDynamics(Cdrag = Cdrag)  annotation(
        Placement(transformation(origin = {-40, 62}, extent = {{-10, -10}, {10, 10}})));
    equation
    //  Fx  =       LongitudinalDynamics.Fx;
    //  Ftraction = LongitudinalDynamics.Ftraction;
    //  Fbrake  =   LongitudinalDynamics.Fbrake;
    //  Fdrag =     LongitudinalDynamics.Fdrag;
    //  Fdrag =     AeroDynamics.Faerodrag;
    //  Ax  =       LongitudinalDynamics.Ax;
    //  Vx  =       LongitudinalDynamics.Vx;
    //  Vx  =       AeroDynamics.Vx;
    end baseMainSimulation;
  
    model simpleMainSimulation 
      extends RaceCarLibrary.SimulationsP.baseMainSimulation(
        carMass=800,
        Cdrag=3,
        Vx(start = 0) 
        );
      parameter Modelica.Units.SI.Force maxBrakingForce = 15000 "Max braking force (N)";
      parameter Modelica.Units.SI.Force maxTractionForce = 5000 "Max traction force (N)";
    equation
      LongitudinalDynamics.Fbrake = Driver.brake  * maxBrakingForce;
      LongitudinalDynamics.Ftraction = Driver.throttle  * maxTractionForce;
  
    end simpleMainSimulation;
    
  end SimulationsP;
annotation(
    uses(Modelica(version = "4.0.0")));
end RaceCarLibrary;
