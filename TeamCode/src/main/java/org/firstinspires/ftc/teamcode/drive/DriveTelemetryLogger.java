package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Telemetry based logging.
 * It logs a message to an item in the telemetry screen, in telemetry log and to the
 * android log.
 *
 * @author Harshal Bharatia
 */
public class DriveTelemetryLogger
{
    private static final String MAIN_TAG = "vibotApp-";
    /**
     * Current state of logging.
     * The state values acts as a bit-mask against which the log-mode bit can be easily compared.
     * E.g. In INFO mode only INFO, ERROR and FATAL mode log attempts must be allowed to log and
     * TRACE, DEBUG modes must not be allowed.
     *
     * @author Harshal Bharatia
     */
    public enum LogState
    {
        TRACE(0x3F),
        DEBUG(0x1F),
        INFO(0x0F),
        WARN(0x07),
        ERROR(0x03),
        FATAL(0x01),
        NONE(0x00),
        ALL(0xFF);

        /**
         * Log state value.
         */
        private int mLogState;

        private LogState(int mode)
        {
            mLogState = mode;
        }

        /**
         * Get the log state value.
         * @return
         */
        public int getLogState()
        {
            return(mLogState);
        }

        /**
         * Check of a logging of specified mode can be performed in the current state.
         *
         * @param mode  Logging mode being checked.
         * @return  True if logging should be allowed.
         */
        public boolean isLoggable(LogMode mode)
        {
            return((mLogState & mode.getLogMode()) != 0);
        }
    };

    /**
     * Mode in which logging is being attempted.
     *
     * @author Harshal Bharatia
     */
    public enum LogMode
    {
        TRACE(0x20, "T"),
        DEBUG(0x10, "D"),
        INFO(0x08, "I"),
        WARN(0x04, "W"),
        ERROR(0x02, "E"),
        FATAL(0x01, "F");

        /**
         * Log mode value.
         */
        private int mLogMode;

        private String mQualifier;

        private LogMode(int state, String qualifier)
        {
            mLogMode = state;
            mQualifier = qualifier;
        }

        public int getLogMode()
        {
            return(mLogMode);
        }

        public String getQualifier()
        {
            return(mQualifier);
        }
    };

    /**
     * Owner of telemetry.
     */
    private OpMode    mOwner;

    /**
     * Telemetry item where logging is performed.
     */
    private Telemetry.Item  mItem;

    /**
     * Tag for logging messages for this logger to Android logging system.
     */
    private String          mTag;

    /**
     * Telemetry log.
     */
    private Telemetry.Log   mLog;

    /**
     * Current log state.
     */
    private LogState        mLogState = LogState.TRACE;

    /**
     * Log counter.
     */
    private long            mCount;

    /**
     * Enable logging to telemetry summary.
     */
    private boolean         mTelemetrySummary  = true;

    /**
     * Enable logging to telemetry log.
     */
    private boolean         mTelemetryLog  = true;

    /**
     * Android logging.
     */
    private boolean         mAndroidLogging = true;

    private TelemetryValue    mValueProvider;

    /**
     * Initialize the telemetry based logging subsystem.
     *
     * @param owner Owner of the telemetry.
     */
    public static void initLoggingSubsystem(OpMode owner)
    {
        // This allows individual items to manage their own values without being cleared
        // at each log attempt.
        owner.telemetry.setAutoClear(false);
    }

    /**
     * Telemetery based logger full constructor.
     *
     * @param owner         Owner linear op-mode.
     * @param caption       Caption where this logger logs its information.
     * @param value         Initial value that is shown for this log item.
     */
    public DriveTelemetryLogger(OpMode owner, String caption, String value)
    {
        mOwner = owner;
        mValueProvider = new TelemetryValue();
        mValueProvider.setValue(value);
        mItem = mOwner.telemetry.addData(caption, mValueProvider);
        mLog = mOwner.telemetry.log();
        mTag = MAIN_TAG + caption;
    }

    /**
     * Telemetery based logger empty value constructor.
     *
     * @param owner         Owner linear op-mode.
     * @param caption       Caption where this logger logs its information.
     */
    public DriveTelemetryLogger(OpMode owner, String caption)
    {
        this(owner, caption, "");
    }

    /**
     * Set the log-state of this telemetry logger.
     * Change this state to increase or decrease the amount of logged information.
     *
     * @param state New state
     * @return  Current instance.
     */
    DriveTelemetryLogger setLogState(LogState state)
    {
        mLogState = state;
        return(this);
    }

    /**
     * Set if log information must also appear in android logging system.
     *
     * @param telemetrySummary  True is log information must apprear in telemetry item.
     * @param telemetryLog  True is log information must apprear in telemetry log.
     * @param androidLogging    True is log information must also apprear in android logging system.
     */
    public void configureLogging(boolean telemetrySummary, boolean telemetryLog, boolean androidLogging)
    {
        mTelemetrySummary = telemetrySummary;
        mTelemetryLog = telemetryLog;
        mAndroidLogging = androidLogging;
    }

    public void clearTelemetry()
    {
        mValueProvider.setValue("");
    }

    /**
     * Get the log state.
     *
     * @return  New log state
     */
    LogState getLogState()
    {
        return(mLogState);
    }

    /**
     * Check is a trace log can be currently logged.
     *
     * @return  True if current log state allows trace logs.
     */
    public boolean isTraceEnabled()
    {
        return(mLogState.isLoggable(LogMode.TRACE));
    }

    /**
     * Check is a debug log can be currently logged.
     *
     * @return  True if current log state allows debug logs.
     */
    public boolean isDebugEnabled()
    {
        return(mLogState.isLoggable(LogMode.DEBUG));
    }

    /**
     * Check is a info log can be currently logged.
     *
     * @return  True if current log state allows info logs.
     */
    public boolean isInfoEnabled()
    {
        return(mLogState.isLoggable(LogMode.INFO));
    }

    /**
     * Check is a warn log can be currently logged.
     *
     * @return  True if current log state allows warn logs.
     */
    public boolean isWarnEnabled()
    {
        return(mLogState.isLoggable(LogMode.WARN));
    }

    /**
     * Check is a error log can be currently logged.
     *
     * @return  True if current log state allows error logs.
     */
    public boolean isErrorEnabled()
    {
        return(mLogState.isLoggable(LogMode.ERROR));
    }

    /**
     * Check is a fatal log can be currently logged.
     *
     * @return  True if current log state allows fatal logs.
     */
    public boolean isFatalEnabled()
    {
        return(mLogState.isLoggable(LogMode.FATAL));
    }

    /**
     * Attempt a trace log.
     *
     * @param value Value to be logged.
     */
    public void trace(Object value)
    {
        if (mLogState.isLoggable(LogMode.TRACE))
        {
            doLog(LogMode.TRACE, value);
        }
    }

    /**
     * Attempt a debug log.
     *
     * @param value Value to be logged.
     */
    public void debug(Object value)
    {
        if (mLogState.isLoggable(LogMode.DEBUG))
        {
            doLog(LogMode.DEBUG, value);
        }
    }

    /**
     * Attempt a info log.
     *
     * @param value Value to be logged.
     */
    public void info(Object value)
    {
        if (mLogState.isLoggable(LogMode.INFO))
        {
            doLog(LogMode.INFO, value);
        }
    }

    /**
     * Attempt a warn log.
     *
     * @param value Value to be logged.
     */
    public void warn(Object value)
    {
        if (mLogState.isLoggable(LogMode.WARN))
        {
            doLog(LogMode.WARN, value);
        }
    }

    /**
     * Attempt a error log.
     *
     * @param value Value to be logged.
     */
    public void error(Object value)
    {
        if (mLogState.isLoggable(LogMode.ERROR))
        {
            doLog(LogMode.ERROR, value);
        }
    }

    /**
     * Attempt a fatal log.
     *
     * @param value Value to be logged.
     */
    public void fatal(Object value)
    {
        if (mLogState.isLoggable(LogMode.FATAL))
        {
            doLog(LogMode.FATAL, value);
        }
    }

    /**
     * Attempt a trace log for specified log-mode.
     *
     * @param mode  Log mode.
     * @param value Value to be logged.
     */
    public void log(LogMode mode, Object value)
    {
        if (mLogState.isLoggable(mode))
        {
            doLog(mode, value);
        }
    }

    /**
     * Do the actual logging.
     *
     * @param value Value to be logged.
     */
    private String doLog(LogMode mode, Object value)
    {
        String message = "" + value;
        boolean updateRequired = false;
        if (mTelemetrySummary)
        {
            mValueProvider.setValue(message);
            //mItem.setValue(message);
            updateRequired = true;
        }

        //if (mTelemetryLog)
        //{
        //    mLog.add(message);
        //    updateRequired = true;
        //}
        if (updateRequired)
        {
            mOwner.telemetry.update();
        }
        if (mAndroidLogging)
        {
            message = mode.getQualifier() + ", " + (++mCount) + " | " + message;
            switch (mode)
            {
            case TRACE:
            {
                if (Log.isLoggable(mTag, Log.INFO))
                {
                    Log.i(mTag, message);
                }
                break;
            }
            case DEBUG:
            {
                if (Log.isLoggable(mTag, Log.INFO))
                {
                    Log.i(mTag, message);
                }
                break;
            }
            case INFO:
            {
                if (Log.isLoggable(mTag, Log.INFO))
                {
                    Log.i(mTag, message);
                }
                break;
            }
            case WARN:
            {
                if (Log.isLoggable(mTag, Log.WARN))
                {
                    Log.w(mTag, message);
                }
                break;
            }
            case ERROR:
            case FATAL:
            {
                if (Log.isLoggable(mTag, Log.ERROR))
                {
                    Log.e(mTag, message);
                }
                break;
            }
            }
        }
        return(message);
    }

    private class TelemetryValue implements Func<String>
    {
        String mValue;

        @Override
        public String value()
        {
            return mValue;
        }

        public void setValue(String value)
        {
            mValue = value;
        }
    }
}