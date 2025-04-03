/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.tasks;

import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements the auto task to operate the Coral Arm and Elevator subsystems.
 */
public class TaskElevatorArm extends TrcAutoTask<TaskElevatorArm.State>
{
    private static final String moduleName = TaskElevatorArm.class.getSimpleName();

    public enum State
    {
        ZERO_CALIBRATE,

        SET_CLIMB_POS,

        SET_CORAL_SCORE_POS,
        CHECK_CORALARM_SAFETY_FOR_SCORE,

        SET_CORAL_STATION_PICKUP_POS,
        CHECK_ELEVATOR_SAFETY_FOR_PICKUP,

        SET_CORALARM_PID_POWER,
        SET_CORALARM_POWER,
        CORALARM_SETPOWER,

        SET_ELEVATOR_PID_POWER,
        SET_ELEVATOR_POWER,
        ELEVATOR_SETPOWER,

        DONE
    }   //enum State

    private static enum Action
    {
        SetCoralScorePosition,
        SetCoralStationPickupPosition,
        SetCoralArmPidPower,
        SetCoralArmPower,
        SetElevatorPidPower,
        SetElevatorPower
    }   //enum Action

    private static class TaskParams
    {
        Action action = null;
        double power = 0.0;
        Double coralArmPos = null;
        Double elevatorPos = null;

        static TaskParams setPositionParams(Action action, Double coralArmPos, Double elevatorPos)
        {
            TaskParams params = new TaskParams();
            params.action = action;
            params.coralArmPos = coralArmPos;
            params.elevatorPos = elevatorPos;

            return params;
        }   //setPositionParams

        static TaskParams setPowerParams(Action action, double power)
        {
            TaskParams params = new TaskParams();
            params.action = action;
            params.power = power;

            return params;
        }  //setPowerParams

        public String toString()
        {
            return "action=" + action +
                   ",power=" + power +
                   ",coralArmPos=" + coralArmPos +
                   ",elevatorPos=" + elevatorPos;
        }   //toString
    }   //class TaskParams

    public final TrcMotor coralArm;
    public final TrcMotor elevator;
    private final TrcEvent coralArmEvent;
    private final TrcEvent elevatorEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param coralArm specifies the coral arm object.
     * @param elevator specifies the elevator object.
     */
    public TaskElevatorArm(TrcMotor coralArm, TrcMotor elevator)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.coralArmEvent = new TrcEvent(CoralArm.Params.SUBSYSTEM_NAME);
        this.elevatorEvent = new TrcEvent(Elevator.Params.SUBSYSTEM_NAME);
    }   //TaskElevatorArm

    /**
     * This method starts the zero calibrate operation. It sets the arms to their safe positions before doing zero
     * calibration on the elevator.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "zeroCalibrate(owner=" + owner + ", event=" + completionEvent + ")");
        startAutoTask(owner, State.ZERO_CALIBRATE, null, completionEvent);
    }   //zeroCalibrate

    /**
     * This method sets the elevator and arms to the positions for climbing. It understands the positions of the arms
     * and elevator and coordinates their movements to avoid them colliding with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void setClimbPosition(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "setClimbPosition(owner=" + owner + ", event=" + completionEvent + ")");
        startAutoTask(owner, State.SET_CLIMB_POS, null, completionEvent);
    }   //setClimbPosition

    /**
     * This method sets the elevator and arms to the positions for scoring a Coral on a Reef branch safely. It
     * understands the positions of the arms and elevator and coordinates their movements to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param scoreLevel specifies the level of the reef branch to score the Coral.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void setCoralScorePosition(String owner, int scoreLevel, TrcEvent completionEvent)
    {
        TaskParams taskParams = TaskParams.setPositionParams(
            Action.SetCoralScorePosition, CoralArm.Params.SCORE_LEVEL_POS[scoreLevel],
            Elevator.Params.SCORE_LEVEL_POS[scoreLevel]);
        tracer.traceInfo(
            moduleName,
            "setCoralScorePosition(owner=" + owner +
            ", taskParams=(" + taskParams +
            "), event=" + completionEvent + ")");
        startAutoTask(owner, State.SET_CORAL_SCORE_POS, taskParams, completionEvent);
    }   //setScoreCoralPosition

    /**
     * This method sets the elevator and arms to the positions for picking up a Coral from the station safely. It
     * understands the positions of the arms and elevator and coordinates their movements to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void setCoralStationPickupPosition(String owner, TrcEvent completionEvent)
    {
        TaskParams taskParams = TaskParams.setPositionParams(
            Action.SetCoralStationPickupPosition, CoralArm.Params.STATION_PICKUP_POS,
            Elevator.Params.STATION_PICKUP_POS);
        tracer.traceInfo(
            moduleName,
            "setCoralStationPickupPosition(owner=" + owner +
            ", taskParams=(" + taskParams +
            "), event=" + completionEvent + ")");
        startAutoTask(owner, State.SET_CORAL_STATION_PICKUP_POS, taskParams, completionEvent);
    }   //setCoralStationPickupPosition

    /**
     * This method is typically called by TeleOp to move the Coral Arm using a joystick. It understands the positions
     * of the arms and elevator and moves them to safe positions before moving the Coral Arm to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param power specifies the power to apply to the Coral Arm motor.
     */
    public void setCoralArmPidPower(String owner, double power)
    {
        if (coralArm != null)
        {
            TaskParams taskParams = TaskParams.setPowerParams(Action.SetCoralArmPidPower, power);
            tracer.traceDebug(moduleName, "setCoralArmPidPower(owner=%s, taskParams=(%s))", owner, taskParams);
            startAutoTask(owner, State.SET_CORALARM_PID_POWER, taskParams, null);
        }
    }   //setCoralArmPidPower

    /**
     * This method is typically called by TeleOp to move the Coral Arm using a joystick. It understands the positions
     * of the arms and elevator and moves them to safe positions before moving the Coral Arm to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param power specifies the power to apply to the Coral Arm motor.
     */
    public void setCoralArmPower(String owner, double power)
    {
        if (coralArm != null)
        {
            TaskParams taskParams = TaskParams.setPowerParams(Action.SetCoralArmPower, power);
            tracer.traceDebug(moduleName, "setCoralArmPower(owner=%s, taskParams=(%s))", owner, taskParams);
            startAutoTask(owner, State.SET_CORALARM_POWER, taskParams, null);
        }
    }   //setCoralArmPower

    /**
     * This method is typically called by TeleOp to move the Elevator using a joystick. It understands the positions
     * of the arms and elevator and moves them to safe positions before moving the Elevator to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param power specifies the power to apply to the Elevator motor.
     */
    public void setElevatorPidPower(String owner, double power)
    {
        // Elevator must exist.
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetElevatorPidPower, power);
        tracer.traceDebug(moduleName, "setElevatorPidPower(owner=%s, taskParams=(%s))", owner, taskParams);
        startAutoTask(owner, State.SET_ELEVATOR_PID_POWER, taskParams, null);
    }   //setElevatorPidPower

    /**
     * This method is typically called by TeleOp to move the Elevator using a joystick. It understands the positions
     * of the arms and elevator and moves them to safe positions before moving the Elevator to avoid them colliding
     * with each other.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param power specifies the power to apply to the Elevator motor.
     */
    public void setElevatorPower(String owner, double power)
    {
        // Elevator must exist.
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetElevatorPower, power);
        tracer.traceDebug(moduleName, "setElevatorPower(owner=%s, taskParams=(%s))", owner, taskParams);
        startAutoTask(owner, State.SET_ELEVATOR_POWER, taskParams, null);
    }   //setCoralArmPidPower

    /**
     * This method determines if the given Coral Arm position is in the safe zone.
     *
     * @param pos specifies the Coral Arm position.
     * @return true if Coral Arm position is in the safe zone, false otherwise.
     */
    private boolean isCoralArmPosInSafeZone(double pos)
    {
        return coralArm == null || pos >= CoralArm.Params.SAFE_ZONE_POS;
    }   //isCoralArmPosInSafeZone

    /**
     * This method determines if the given Elevator position is in the safe zone.
     *
     * @param pos specifies the Elevator position.
     * @return true if Elevator position is in the safe zone, false otherwise.
     */
    private boolean isElevatorPosInSafeZone(double pos)
    {
        // Elevator must exist.
        return pos <= Elevator.Params.SAFE_ZONE_POS;
    }   //isElevatorPosInSafeZone

    /**
     * This method determines if it is safe to move the Coral Arm. Moving the arm to the safe zone is always safe
     * because if arm is currently in unsafe zone, elevator must be in its safe zone. However, moving the arm to
     * unsafe zone requires the elevator to be in its safe zone.
     *
     * @param targetPos specifies the Coral Arm target position.
     * @return true if it is safe to move the Coral Arm, false otherwise.
     */
    private boolean isSafeToMoveCoralArm(double targetPos)
    {
        return isCoralArmPosInSafeZone(targetPos) || isElevatorPosInSafeZone(elevator.getPosition());
    }   //isSafeToMoveCoralArm

    /**
     * This method determines if it is safe to move the Elevator. Moving the elevator to the safe zone is always safe
     * because if elevator is currently in unsafe zone, both arms must be in their safe zones. However, moving the
     * elevator to unsafe zone requires both arms to be in their safe zones.
     *
     * @param targetPos specifies the Elevator target position.
     * @return true if it is safe to move the Elevator, false otherwise.
     */
    private boolean isSafeToMoveElevator(double targetPos)
    {
        return isElevatorPosInSafeZone(targetPos) || coralArm == null ||
               isCoralArmPosInSafeZone(coralArm.getPosition());
    }   //isSafeToMoveElevator

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null ||
               (coralArm == null || coralArm.acquireExclusiveAccess(owner)) &&
               elevator.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\tcoralArmOwner=" + (coralArm == null? "null": ownershipMgr.getOwner(coralArm)) +
                "\n\televatorOwner=" + ownershipMgr.getOwner(elevator));
            if (coralArm != null) coralArm.releaseExclusiveAccess(owner);
            elevator.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        // Do not cancel elevator or arm because we need them to hold position.
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;
        double targetPos;

        switch (state)
        {
            //
            // Zero Calibrate.
            //
            case ZERO_CALIBRATE:
                coralArmEvent.clear();
                elevatorEvent.clear();
                if (coralArm != null)
                {
                    coralArm.setPosition(
                        owner, 0.0, CoralArm.Params.SAFE_ZONE_POS, true, CoralArm.Params.POWER_LIMIT, coralArmEvent,
                        0.0);
                    sm.addEvent(coralArmEvent);
                }
                elevator.zeroCalibrate(owner, Elevator.Params.ZERO_CAL_POWER, elevatorEvent);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(State.DONE, false, true);
                break;
            //
            // Set Climb Position.
            //
            case SET_CLIMB_POS:
                coralArmEvent.clear();
                elevatorEvent.clear();
                if (coralArm != null)
                {
                    coralArm.setPosition(
                        owner, 0.0, CoralArm.Params.CLIMB_POS, true, CoralArm.Params.POWER_LIMIT, coralArmEvent, 0.0);
                    sm.addEvent(coralArmEvent);
                }
                elevator.setPosition(
                    owner, 0.0, Elevator.Params.CLIMB_POS, true, Elevator.Params.POWER_LIMIT, elevatorEvent, 0.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(State.DONE, false, true);
                break;
            //
            // Set Coral Score Position.
            //
            case SET_CORAL_SCORE_POS:
                coralArmEvent.clear();
                if (coralArm != null)
                {
                    coralArm.setPosition(
                        owner, 0.0, taskParams.coralArmPos, true, CoralArm.Params.POWER_LIMIT, coralArmEvent, 0.0);
                    // sm.addEvent(coralArmEvent);
                }
                sm.setState(State.CHECK_CORALARM_SAFETY_FOR_SCORE);
                break;

            case CHECK_CORALARM_SAFETY_FOR_SCORE:
                if (coralArm == null || isCoralArmPosInSafeZone(coralArm.getPosition()))
                {
                    elevator.setPosition(
                        owner, 0.0, taskParams.elevatorPos, true, Elevator.Params.POWER_LIMIT, elevatorEvent, 0.0);
                    // sm.addEvent(elevatorEvent);
                    sm.waitForSingleEvent(elevatorEvent, State.DONE);//, false, true);
                }
                break;
            //
            // Set Coral Station Pickup Position.
            //
            case SET_CORAL_STATION_PICKUP_POS:
                elevatorEvent.clear();
                elevator.setPosition(
                    owner, 0.0, taskParams.elevatorPos, true, Elevator.Params.POWER_LIMIT, elevatorEvent, 0.0);
                // coralArm.setPosition(owner, 0.0, CoralArm.Params.SAFE_ZONE_POS, true, CoralArm.Params.POWER_LIMIT, null, 0.0);
                sm.addEvent(elevatorEvent);
                sm.setState(State.CHECK_ELEVATOR_SAFETY_FOR_PICKUP);
                break;

            case CHECK_ELEVATOR_SAFETY_FOR_PICKUP:
                if (isElevatorPosInSafeZone(elevator.getPosition()))
                {
                    if (coralArm != null)
                    {
                        coralArm.setPosition(
                            owner, 0.0, taskParams.coralArmPos, true, CoralArm.Params.POWER_LIMIT, coralArmEvent, 0.0);
                        sm.addEvent(coralArmEvent);
                    }
                    sm.waitForEvents(State.DONE, false, true);
                }
                break;
            //
            // Set Coral Arm Power.
            //
            case SET_CORALARM_PID_POWER:
            case SET_CORALARM_POWER:
                targetPos = taskParams.power > 0.0? CoralArm.Params.MAX_POS: CoralArm.Params.MIN_POS;
                if (taskParams.power != 0.0 && !isSafeToMoveCoralArm(targetPos))
                {
                    elevator.setPosition(
                        owner, 0.0, Elevator.Params.SAFE_ZONE_POS, true, Elevator.Params.POWER_LIMIT, elevatorEvent,
                        0.0);
                    sm.waitForSingleEvent(elevatorEvent, State.CORALARM_SETPOWER);
                }
                else
                {
                    sm.setState(State.CORALARM_SETPOWER);
                }
                break;

            case CORALARM_SETPOWER:
                if (coralArm != null)
                {
                    if (taskParams.action == Action.SetCoralArmPidPower)
                    {
                        coralArm.setPidPower(
                            owner, taskParams.power, CoralArm.Params.MIN_POS, CoralArm.Params.MAX_POS, true);
                    }
                    else
                    {
                        coralArm.setPower(owner, 0.0, taskParams.power, 0.0, null);
                    }
                }
                sm.setState(State.DONE);
                break;
            //
            // Set Elevator Power.
            //
            case SET_ELEVATOR_PID_POWER:
            case SET_ELEVATOR_POWER:
                targetPos = taskParams.power > 0.0? Elevator.Params.MAX_POS: Elevator.Params.MIN_POS;
                if (taskParams.power != 0.0 && !isSafeToMoveElevator(targetPos))
                {
                    // CoralArm has backlash and will oscillate a little at the end.
                    // We don't want to wait for it to settle, so enable noOscillation to move on to the next state.
                    coralArm.getPosPidCtrl().setNoOscillation(true);
                    coralArm.setPosition(
                        owner, 0.0, CoralArm.Params.SAFE_ZONE_POS, true, CoralArm.Params.POWER_LIMIT, coralArmEvent,
                        0.0);
                    sm.waitForSingleEvent(coralArmEvent, State.ELEVATOR_SETPOWER);
                }
                else
                {
                    sm.setState(State.ELEVATOR_SETPOWER);
                }
                break;

            case ELEVATOR_SETPOWER:
                if (coralArm != null)
                {
                    coralArm.getPosPidCtrl().setNoOscillation(false);
                }

                if (taskParams.action == Action.SetElevatorPidPower)
                {
                    elevator.setPidPower(owner, taskParams.power, Elevator.Params.MIN_POS, Elevator.Params.MAX_POS, true);
                }
                else
                {
                    elevator.setPower(owner, 0.0, taskParams.power, 0.0, null);
                }
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskElevatorArm
