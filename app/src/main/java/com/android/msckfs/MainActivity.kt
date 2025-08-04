package com.android.msckfs

import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.android.msckfs.imuProcessing.ImuMessage
import com.android.msckfs.imuProcessing.ImuProcessor
import com.android.msckfs.msckfs.Msckf
import java.util.function.Consumer

class MainActivity : AppCompatActivity() {

    private var imuProcessor: ImuProcessor? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        // VIO
        // TODO: coroutines?
        val msckf = Msckf()
        //val callback = { msg : ImuMessage -> msckf.imuCallback(msg) } // TODO: https://www.baeldung.com/kotlin/lambda-expressions
        imuProcessor = ImuProcessor(applicationContext, msckf)
        /*
        setContentView(R.layout.activity_main)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }

         */
    }


    override fun onDestroy() { // TODO: onDestroy? or onStop?
        super.onDestroy()
        imuProcessor?.stop()
    }


}