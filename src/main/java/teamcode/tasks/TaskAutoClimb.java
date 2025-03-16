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

import teamcode.Robot;
import teamcode.subsystems.Winch;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements Auto Climb task.
 */
public class TaskAutoClimb extends TrcAutoTask<TaskAutoClimb.State>
{
    private static final String moduleName = TaskAutoClimb.class.getSimpleName();

    public enum State
    {
        DEPLOY_CLIMBER,
        FINISH_DEPLOY_CLIMBER,
        PREP_CLIMBER,
        CLIMB,
        DONE
    }   //enum State

    private final Robot robot;
    private final TrcEvent winchEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoClimb(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.winchEvent = new TrcEvent(moduleName + ".winchEvent");
    }   //TaskAutoClimb

    /**
     * This method deploys the climber by zero calibrating it and extending it out.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void deployClimber(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "owner=" + owner + ", event=" + completionEvent);
        startAutoTask(owner, State.DEPLOY_CLIMBER, null, completionEvent);
    }   //deployClimber

    /**
     * This method prepares the climber to climb.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void prepClimber(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "owner=" + owner + ", event=" + completionEvent);
        startAutoTask(owner, State.PREP_CLIMBER, null, completionEvent);
    }   //prepClimber

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void climb(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "owner=" + owner + ", event=" + completionEvent);
        startAutoTask(owner, State.CLIMB, null, completionEvent);
    }   //climb

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
        return owner == null || robot.winch.acquireExclusiveAccess(owner);
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
                "\n\twinch=" + ownershipMgr.getOwner(robot.winch));
            robot.winch.releaseExclusiveAccess(owner);
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
        robot.winch.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
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
        switch (state)
        {
            case DEPLOY_CLIMBER:
                robot.winch.zeroCalibrate(owner, Winch.Params.ZERO_CAL_POWER, winchEvent);
                sm.waitForSingleEvent(winchEvent, State.FINISH_DEPLOY_CLIMBER);
                break;

            case FINISH_DEPLOY_CLIMBER:
                robot.winch.setPosition(owner, 0.0, Winch.Params.DEPLOY_POS, true, 1.0, winchEvent, 0.0);
                sm.waitForSingleEvent(winchEvent, State.DONE);
                break;

            case PREP_CLIMBER:
                robot.winch.setPosition(owner, 0.0, Winch.Params.PRE_CLIMB_POS, true, 1.0, winchEvent, 0.0);
                robot.elevatorArmTask.coralArm.setPosition(90.0);
                sm.waitForSingleEvent(winchEvent, State.DONE);
                break;

            case CLIMB:
                robot.winch.setPosition(owner, 0.0, Winch.Params.CLIMB_POS, true, 0.5, winchEvent, 0.0);
                sm.waitForSingleEvent(winchEvent, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoClimb
