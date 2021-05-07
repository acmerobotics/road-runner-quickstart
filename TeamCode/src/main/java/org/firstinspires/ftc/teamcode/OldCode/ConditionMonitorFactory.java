package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Condition monitor factory that provides implementation of some common monitor conditions.
 *
 * @author Harshal Bharatia
 */
@Disabled
public class ConditionMonitorFactory
{
    private static volatile ConditionMonitorFactory mInstance;

    /**
     * Get the singleton instance of this factory.
     *
     * @return  Singleton instance of this factory.
     */
    public static ConditionMonitorFactory getInstance()
    {
        if (mInstance != null)
        {
            return(mInstance);
        }
        synchronized (ConditionMonitorFactory.class)
        {
            if (mInstance != null)
            {
                return(mInstance);
            }
            mInstance = new ConditionMonitorFactory();
            return(mInstance);
        }
    }

    private ConditionMonitorFactory()
    {
    }

    /**
     * Monitor that stops operation if it exceeds a specific time.
     *
     * @author Harshal Bharatia
     */
    public class TimeoutMonitor extends ElapsedTime
        implements ConditionMonitor
    {
        /**
         * Timeout millis.
         */
        private long        mTimeoutMillis;

        /**
         * Represents if monitoring the timeout had started.
         */
        private boolean     mStarted;

        /**
         * Constructor.
         *
         * @param timeoutMillis     Timeout milliseconds.
         */
        public TimeoutMonitor(long timeoutMillis)
        {
            mTimeoutMillis = timeoutMillis;
        }

        @Override
        public boolean canContinue()
        {
            assertStarted();
            return(milliseconds() < mTimeoutMillis);
        }

        /**
         * Ensure that timer has been started.
         */
        private void assertStarted()
        {
            if (!mStarted)
            {
                startMonitor();
            }
        }

        /**
         * Start the monitor.
         */
        public void startMonitor()
        {
            reset();
            mStarted = true;
        }

        /**
         * Clear the monitor so that it can be used again.
         */
        public TimeoutMonitor clearMonitor()
        {
            mStarted = false;
            return(this);
        }

        /**
         * Clear the monitor so that it can be used again.
         */
        public TimeoutMonitor clearMonitor(long newTimeoutMillis)
        {
            mStarted = false;
            mTimeoutMillis = newTimeoutMillis;
            return(this);
        }
    }

    /**
     * Monitor that stops operation if the specified input is no longer in required state.
     *
     * @author Harshal Bharatia
     */
    public class InputMonitor implements ConditionMonitor
    {
        private TouchSensor mInput;

        /**
         * State that qualifies to continue operation.
         */
        private boolean         mInputState = false;

        /**
         * Constructor to continue until input-state remains fails.
         *
         * @param input             Input sensor.
         */
        public InputMonitor(TouchSensor input)
        {
            mInput = input;
        }

        /**
         * Constructor to continue until input-state remains as specified.
         *
         * @param input             Input sensor.
         * @param continueInputState   Continue input state.
         */
        public InputMonitor(TouchSensor input, boolean continueInputState)
        {
            mInput = input;
            mInputState = continueInputState;
        }

        @Override
        public boolean canContinue()
        {
            return(mInput.isPressed() == mInputState);
        }

        /**
         * Clear the monitor so that it can be used again.
         *
         * @param continueInputState New continue input state.
         */
        public InputMonitor clearMonitor(boolean continueInputState)
        {
            mInputState = continueInputState;
            return(this);
        }
    }

    /**
     * Monitor that stops operation if it exceeds a specific time OR the specified input is no
     * longer in required state.
     *
     * @author Harshal Bharatia
     */
    public class InputTimeoutMonitor extends TimeoutMonitor
    {
        private TouchSensor mInput;

        /**
         * State that qualifies to continue operation.
         */
        private boolean         mInputState = false;

        private TelemetryLogger mLogger;

        /**
         * Constructor to continue until input-state remains fails.
         *
         * @param input             Input sensor.
         * @param timeoutMillis     Timeout milliseconds.
         */
        public InputTimeoutMonitor(TouchSensor input, long timeoutMillis)
        {
            super(timeoutMillis);
            mInput = input;
        }

        public void setLogger(TelemetryLogger logger)
        {
            mLogger = logger;
        }

        /**
         * Constructor to continue until input-state remains as specified.
         *
         * @param timeoutMillis     Timeout milliseconds.
         * @param input             Input sensor.
         * @param continueInputState   Continue input state.
         */
        public InputTimeoutMonitor(long timeoutMillis, TouchSensor input, boolean continueInputState)
        {
            super(timeoutMillis);
            mInput = input;
            mInputState = continueInputState;
        }

        @Override
        public boolean canContinue()
        {
            //if (mLogger == null || !mLogger.isTraceEnabled())
            //{
                return(super.canContinue() && mInput.isPressed() == mInputState);
            //}
            //else
            //{
            //    boolean notTimeout = super.canContinue();
            //    boolean isPressed = mInput.isPressed();
            //    mLogger.trace("TimeoutPress: timeout=" + (!notTimeout) + ", pressed=" + isPressed);
            //    return(notTimeout && isPressed == mInputState);
            //}
        }

        /**
         * Clear the monitor so that it can be used again.
         *
         * @param newTimeoutMillis  New timeout.
         * @param continueInputState New continue input state.
         */
        public InputTimeoutMonitor clearMonitor(long newTimeoutMillis, boolean continueInputState)
        {
            super.clearMonitor(newTimeoutMillis);
            mInputState = continueInputState;
            return(this);
        }
    }

    /**
     * Composite monitor that allows monitoring based on a collection of conditions.
     *
     * @author Harshal Bharatia
     */
    public class CompositeMonitor implements ConditionMonitor
    {
        /**
         * List of monitors.
         */
        private List<ConditionMonitor> mMonitorList;

        /**
         * Constructor with empty monitor list.
         */
        public CompositeMonitor()
        {
            mMonitorList = new ArrayList<ConditionMonitor>();
        }

        /**
         * Constructor with variable number of monitors.
         * @param monitors  Monitor variable arguments.
         */
        public CompositeMonitor(ConditionMonitor ... monitors)
        {
            mMonitorList = new ArrayList<ConditionMonitor>(monitors.length);
            if (monitors.length > 0)
            {
                for (ConditionMonitor monitor: monitors)
                {
                    if (monitor != null)
                    {
                        mMonitorList.add(monitor);
                    }
                }
            }
        }

        /**
         * Constructor with list of monitors.
         *
         * @param monitorList   List of monitors.
         */
        public CompositeMonitor(List<ConditionMonitor> monitorList)
        {
            int numMonitor = (monitorList != null) ? monitorList.size() : 0;
            mMonitorList = new ArrayList<ConditionMonitor>(numMonitor);
            if (numMonitor > 0)
            {
                for (ConditionMonitor monitor: monitorList)
                {
                    if (monitor != null)
                    {
                        mMonitorList.add(monitor);
                    }
                }
            }
        }

        /**
         * Copy constructor.
         *
         * @param other Other composite monitor.
         */
        public CompositeMonitor(CompositeMonitor other)
        {
            this(other.mMonitorList);
        }

        /**
         * Add a new monitor to this instance.
         *
         * @param monitor   New monitor being added.
         */
        public void addMonitor(ConditionMonitor monitor)
        {
            mMonitorList.add(monitor);
        }

        /**
         * Add an existing monitor from this instance.
         *
         * @param monitor   Monitor being removed.
         */
        public void removeMonitor(ConditionMonitor monitor)
        {
            mMonitorList.remove(monitor);
        }

        @Override
        public boolean canContinue()
        {
            int numMonitor = mMonitorList.size();
            if (numMonitor > 0)
            {
                // Continue only if all monitors allow to continue.
                for (int i = 0; i < numMonitor; i++)
                {
                    ConditionMonitor monitor = mMonitorList.get(i);
                    if (!monitor.canContinue())
                    {
                        return(false);
                    }
                }
            }
            return(true);
        }
    }

    /**
     * Create a new timeout monitor.
     *
     * @param timeoutMillis     Timeout millis.
     * @return  Timeout monitor.
     */
    public TimeoutMonitor newTimeoutMonitor(long timeoutMillis)
    {
        return(new TimeoutMonitor(timeoutMillis));
    }

    /**
     * Create a new input monitor.
     *
     * @param input     Digital input channel.
     * @return  Input monitor.
     */
    public InputMonitor newInputMonitor(TouchSensor input)
    {
        return(new InputMonitor(input));
    }

    /**
     * Create monitor that allows monitoring an input and times out if no input is received
     * within specific time.
     *
     * @param input     Digital input channel.
     * @param timeoutMillis     Timeout millis.
     * @return  Input-timeout monitor.
     */
    public InputTimeoutMonitor newInputTimeoutMonitor(TouchSensor input, long timeoutMillis)
    {
        return(new InputTimeoutMonitor(input, timeoutMillis));
    }

    /**
     * Create a composite monitor that allows monitoring for all specified conditions.
     *
     * @param monitors     Zero or more monitors.
     * @return  Composite monitor.
     */
    public CompositeMonitor newCompositeMonitor(ConditionMonitor ... monitors)
    {
        return(new CompositeMonitor(monitors));
    }

    /**
     * Create a composite monitor that allows monitoring for all specified conditions.
     *
     * @param monitorList  List of monitors.
     * @return  Composite monitor.
     */
    public CompositeMonitor newCompositeMonitor(List<ConditionMonitor> monitorList)
    {
        return(new CompositeMonitor(monitorList));
    }
}