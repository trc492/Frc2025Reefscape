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

package teamcode.autotasks;

import teamcode.subsystems.AlgaeArm;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements the auto task to operate the Coral Arm, Algae Arm and Elevator subsystems.
 */
public class TaskElevatorArm extends TrcAutoTask<TaskElevatorArm.State>
{
    private static final String moduleName = TaskElevatorArm.class.getSimpleName();

    public enum State
    {
        ZERO_CALIBRATE,

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
        ZeroCalibrate,
        SetCoralScorePosition,
        SetCoralStationPickupPosition,
        SetCoralArmPidPower,
        SetCoralArmPower,
        SetAlgaeArmPidPower,
        SetAlgaeArmPower,
        SetElevatorPidPower,
        SetElevatorPower
    }   //enum Action

    private static class TaskParams
    {
        Action action = null;
        double power = 0.0;
        Double coralArmPos = null;
        Double algaeArmPos = null;
        Double elevatorPos = null;

        static TaskParams setPositionParams(Action action, Double coralArmPos, Double algaeArmPos, Double elevatorPos)
        {
            TaskParams params = new TaskParams();
            params.action = action;
            params.coralArmPos = coralArmPos;
            params.algaeArmPos = algaeArmPos;
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
                   ",algaeArmPos=" + algaeArmPos +
                   ",elevatorPos=" + elevatorPos;
        }   //toString
    }   //class TaskParams

    public final TrcMotor coralArm;
    public final TrcMotor algaeArm;
    public final TrcMotor elevator;
    private final TrcEvent coralArmEvent;
    private final TrcEvent algaeArmEvent;
    private final TrcEvent elevatorEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param coralArm specifies the coral arm object.
     * @param algaeArm specifies the algae arm object.
     * @param elevator specifies the elevator object.
     */
    public TaskElevatorArm(TrcMotor coralArm, TrcMotor algaeArm, TrcMotor elevator)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.coralArm = coralArm;
        this.algaeArm = algaeArm;
        this.elevator = elevator;
        this.coralArmEvent = new TrcEvent(CoralArm.Params.SUBSYSTEM_NAME);
        this.algaeArmEvent = new TrcEvent(AlgaeArm.Params.SUBSYSTEM_NAME);
        this.elevatorEvent = new TrcEvent(Elevator.Params.SUBSYSTEM_NAME);
    }   //TaskElevatorArm

    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "zeroCalibrate(owner=" + owner + ", event=" + completionEvent + ")");
        startAutoTask(owner, State.ZERO_CALIBRATE, null, completionEvent);
    }   //zeroCalibrate

    public void setCoralScorePositions(String owner, int scoreLevel, TrcEvent completionEvent)
    {
        TaskParams taskParams = TaskParams.setPositionParams(
            Action.SetCoralScorePosition, CoralArm.Params.SCORE_LEVEL_POS[scoreLevel], null,
            Elevator.Params.SCORE_LEVEL_POS[scoreLevel]);
        tracer.traceInfo(
            moduleName,
            "setCoralScorePosition(owner=" + owner +
            ", taskParams=(" + taskParams +
            "), event=" + completionEvent + ")");
        startAutoTask(owner, State.SET_CORAL_SCORE_POS, taskParams, completionEvent);
    }   //setScoreCoralPositions

    public void setCoralStationPickupPositions(String owner, TrcEvent completionEvent)
    {
        TaskParams taskParams = TaskParams.setPositionParams(
            Action.SetCoralStationPickupPosition, CoralArm.Params.STATION_PICKUP_POS, null,
            Elevator.Params.STATION_PICKUP_POS);
        tracer.traceInfo(
            moduleName,
            "setCoralStationPickupPosition(owner=" + owner +
            ", taskParams=(" + taskParams +
            "), event=" + completionEvent + ")");
        startAutoTask(owner, State.SET_CORAL_STATION_PICKUP_POS, taskParams, completionEvent);
    }   //setCoralStationPickupPositions

    public void setCoralArmPidPower(String owner, double power)
    {
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetCoralArmPidPower, power);
        tracer.traceInfo(
            moduleName,
            "setCoralArmPidPower(owner=" + owner +
            ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.SET_CORALARM_PID_POWER, taskParams, null);
    }   //setCoralArmPidPower

    public void setCoralArmPower(String owner, double power)
    {
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetCoralArmPower, power);
        tracer.traceInfo(
            moduleName,
            "setCoralArmPower(owner=" + owner +
            ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.SET_CORALARM_POWER, taskParams, null);
    }   //setCoralArmPower

    public void setElevatorPidPower(String owner, double power)
    {
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetElevatorPidPower, power);
        tracer.traceInfo(
            moduleName,
            "setElevatorPidPower(owner=" + owner +
            ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.SET_ELEVATOR_PID_POWER, taskParams, null);
    }   //setElevatorPidPower

    public void setElevatorPower(String owner, double power)
    {
        TaskParams taskParams = TaskParams.setPowerParams(Action.SetElevatorPower, power);
        tracer.traceInfo(
            moduleName,
            "setElevatorPower(owner=" + owner +
            ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.SET_ELEVATOR_POWER, taskParams, null);
    }   //setCoralArmPidPower

    private boolean isCoralArmPosInSafeZone(double pos)
    {
        return pos >= CoralArm.Params.SAFE_ZONE_POS;
    }   //isCoralArmPosInSafeZone

    private boolean isAlgaeArmPosInSafeZone(double pos)
    {
        return pos >= AlgaeArm.Params.SAFE_ZONE_POS;
    }   //isAlgaeArmPosInSafeZone

    private boolean isElevatorPosInSafeZone(double pos)
    {
        return pos <= Elevator.Params.SAFE_ZONE_POS;
    }   //isElevatorPosInSafeZone

    private boolean isSafeToMoveCoralArm(double targetPos)
    {
        // Moving the arm to the safe zone is always safe because if arm is currently in unsafe zone,
        // elevator must be in its safe zone.
        // Moving the arm to unsafe zone requires the elevator to be in its safe zone.
        return isCoralArmPosInSafeZone(targetPos) || isElevatorPosInSafeZone(elevator.getPosition());
    }   //isSafeToMoveCoralArm

    private boolean isSafeToMoveAlgaeArm(double targetPos)
    {
        // Moving the arm to the safe zone is always safe because if arm is currently in unsafe zone,
        // elevator must be in its safe zone.
        // Moving the arm to unsafe zone requires the elevator to be in its safe zone.
        return isAlgaeArmPosInSafeZone(targetPos) || isElevatorPosInSafeZone(elevator.getPosition());
    }   //isSafeToMoveAlgaeArm

    private boolean isSafeToMoveElevator(double targetPos)
    {
        // Moving elevator to the safe zone is always safe because if elevator is currently in unsafe zone,
        // both arms must be in their safe zone.
        // Moving the elevator to unsafe zone requires both arms to be in their safe zones.
        return isElevatorPosInSafeZone(targetPos) ||
               isCoralArmPosInSafeZone(coralArm.getPosition()) && isAlgaeArmPosInSafeZone(algaeArm.getPosition());
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
        return coralArm.acquireExclusiveAccess(owner) &&
               algaeArm.acquireExclusiveAccess(owner) &&
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
        TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
        tracer.traceInfo(
            moduleName,
            "Releasing subsystem ownership on behalf of " + owner +
            "\n\tcoralArmOwner=" + ownershipMgr.getOwner(coralArm) +
            "\n\talgaeArmOwner=" + ownershipMgr.getOwner(algaeArm) +
            "\n\televatorOwner=" + ownershipMgr.getOwner(elevator));
        coralArm.releaseExclusiveAccess(owner);
        algaeArm.releaseExclusiveAccess(owner);
        elevator.releaseExclusiveAccess(owner);
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
        coralArm.cancel();
        algaeArm.cancel();
        elevator.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto task.
     *
     * @param owner specifies the owner acquired subsystem ownerships.
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
            case ZERO_CALIBRATE:
                coralArmEvent.clear();
                elevatorEvent.clear();
                coralArm.setPosition(
                    owner, 0.0, CoralArm.Params.SAFE_ZONE_POS, true, CoralArm.Params.POWER_LIMIT, coralArmEvent,
                    0.0);
                sm.addEvent(coralArmEvent);
                elevator.zeroCalibrate(owner, Elevator.Params.ZERO_CAL_POWER, elevatorEvent);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(State.DONE, false, true);
                break;

            case SET_CORAL_SCORE_POS:
                coralArmEvent.clear();
                coralArm.setPosition(
                    owner, 0.0, taskParams.coralArmPos, true, CoralArm.Params.POWER_LIMIT, coralArmEvent, 0.0);
                sm.addEvent(coralArmEvent);
                sm.setState(State.CHECK_CORALARM_SAFETY_FOR_SCORE);
                break;

            case CHECK_CORALARM_SAFETY_FOR_SCORE:
                if (isCoralArmPosInSafeZone(coralArm.getPosition()))
                {
                    elevator.setPosition(
                        owner, 0.0, taskParams.elevatorPos, true, Elevator.Params.POWER_LIMIT, elevatorEvent, 0.0);
                    sm.addEvent(elevatorEvent);
                    sm.waitForEvents(State.DONE, false, true);
                }
                break;

            case SET_CORAL_STATION_PICKUP_POS:
                break;

            case CHECK_ELEVATOR_SAFETY_FOR_PICKUP:
                break;

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
                if (taskParams.action == Action.SetCoralArmPidPower)
                {
                    coralArm.setPidPower(owner, taskParams.power, CoralArm.Params.MIN_POS, CoralArm.Params.MAX_POS, true);
                }
                else
                {
                    coralArm.setPower(owner, 0.0, taskParams.power, 0.0, null);
                }
                break;

            case SET_ELEVATOR_PID_POWER:
            case SET_ELEVATOR_POWER:
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

            case ELEVATOR_SETPOWER:
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskElevatorArm
