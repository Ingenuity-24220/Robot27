package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes one CSV file per OpMode run to /sdcard/FIRST/data on the Control Hub.
 * Pull the files over (wireless) ADB with:  adb pull /sdcard/FIRST/data/ ./logs/
 * Logging never throws: if the file can't be written, rows are silently dropped.
 */
public class DataLogger {
    public static final File DIR = new File("/sdcard/FIRST/data");

    private final File file;
    private BufferedWriter out;
    private final StringBuilder row = new StringBuilder(256);

    public DataLogger(String prefix, String... columns) {
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        file = new File(DIR, prefix + "_" + stamp + ".csv");
        try {
            //noinspection ResultOfMethodCallIgnored
            DIR.mkdirs();
            out = new BufferedWriter(new FileWriter(file));
            for (int i = 0; i < columns.length; i++) {
                if (i > 0) out.write(',');
                out.write(columns[i]);
            }
            out.newLine();
        } catch (IOException e) {
            out = null;
        }
    }

    /** Appends one row; pass values in the same order as the columns. */
    public void log(double... values) {
        if (out == null) return;
        row.setLength(0);
        for (int i = 0; i < values.length; i++) {
            if (i > 0) row.append(',');
            row.append(String.format(Locale.US, "%.4f", values[i]));
        }
        try {
            out.write(row.toString());
            out.newLine();
        } catch (IOException e) {
            close();
        }
    }

    public void close() {
        if (out == null) return;
        try {
            out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    /** Path on the Control Hub, or an error note if the file couldn't be opened. */
    public String path() {
        return out != null ? file.getPath() : "NOT LOGGING (" + file.getPath() + ")";
    }
}
