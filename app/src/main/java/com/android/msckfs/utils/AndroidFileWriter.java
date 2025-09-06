package com.android.msckfs.utils;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.msckf.lib.msckfs.Odometry;
import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

public class AndroidFileWriter {


    private static final String TAG = "AndroidFileWriter";

    public AndroidFileWriter(Context context) {
        // TODO: PendingIntent pendingIntent = MediaStore.createWriteRequest(context.getContentResolver(), );
    }
    /**
     * sources:
     * <a href="https://stackoverflow.com/questions/51565897/saving-files-in-android-for-beginners-internal-external-storage">...</a>
     * <a href="https://stackoverflow.com/questions/27772011/how-to-export-data-to-csv-file-in-android">...</a>
     */
    public void writeFileExternalStorage(List<String[]> data) {

        //Checking the availability state of the External Storage.
        String state = Environment.getExternalStorageState();
        if (!Environment.MEDIA_MOUNTED.equals(state)) {

            //If it isn't mounted - we can't write into it. // TODO: what means?
            return;
        }

        String baseDir = android.os.Environment.getExternalStorageDirectory().getAbsolutePath();
        String fileName = "AnalysisData.csv";
        String filePath = baseDir + File.separator + fileName;
        CSVWriter writer;

        try {
            writer = new CSVWriter( new FileWriter(filePath, false)); // TODO: if not "append", does file pre-existing get deleted or just partially overwritten?
            writer.writeAll(data);
        } catch (Exception e) {
            Log.e(TAG, "Could not write to file.", e);
        }


    }

    public void write(List<Odometry> odoms) {
        List<String[]> data = new ArrayList<>(odoms.size()+1);
        data.add(Odometry.getHeadline());
        for (Odometry o : odoms) {
            data.add(o.toStringArray());
        }
        writeFileExternalStorage(data);

    }

}
