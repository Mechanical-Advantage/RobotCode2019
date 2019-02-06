package frc.robot.util;
//the folder this is in is misc. and names BadLog files/gets timestamps.
//when changing this file's location, remember to fix the import around
//Robot.java line 21. Sorry for the inconvinience.
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class LogUtil {

	// RFC2822
	private static SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("EEE', 'dd' 'MMM' 'yyyy' 'HH_mm_ss' 'Z",//creates the date from the device.
			Locale.US);

	public static String getTimestamp() {
		return DATE_FORMAT.format(new Date());
	}

	public static String genSessionName() {
		// Avoids having to check if a file exists
        // Unlikely to cause a collision
        // Used to name randomly, but that's annoying for organization. Now uses date.
		String name = getTimestamp();
		return name;
	}
}