package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;
import org.firstinspires.ftc.teamcode.drive.opmode.TrackWidthTuner2;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public final class TuningWebRoutes {
    private static WebHandler newStaticAssetHandler(AssetManager assetManager, String file) {
        return new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session)
                    throws IOException {
                if (session.getMethod() == NanoHTTPD.Method.GET) {
                    String mimeType = MimeTypesUtil.determineMimeType(file);
                    return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                            mimeType, assetManager.open(file));
                } else {
                    return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                            NanoHTTPD.MIME_PLAINTEXT, "");
                }
            }
        };
    }

    private static void addAssetWebHandlers(WebHandlerManager webHandlerManager,
                                     AssetManager assetManager, String path) {
        try {
            String[] list = assetManager.list(path);

            if (list == null) return;

            if (list.length > 0) {
                for (String file : list) {
                    addAssetWebHandlers(webHandlerManager, assetManager, path + "/" + file);
                }
            } else {
                webHandlerManager.register(path, newStaticAssetHandler(assetManager, path));
            }
        } catch (IOException e) {
            RobotLog.setGlobalErrorMsg(new RuntimeException(e),
                    "unable to register tuning web routes");
        }
    }

    @WebHandlerRegistrar
    public static void registerRoutes(Context context, WebHandlerManager manager) {
        AssetManager assetManager = context.getAssets();
        manager.register("/tuning/trackwidth",
                newStaticAssetHandler(assetManager, "tuning/index.html"));
        manager.register("/tuning/trackwidth/",
                newStaticAssetHandler(assetManager, "tuning/index.html"));

        manager.register("/tuning/trackwidth/latest.json", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws IOException, NanoHTTPD.ResponseException {
                File[] files = TrackWidthTuner2.TRACK_WIDTH_DIR.listFiles();
                if (files != null) {
                    long mostRecentLastModified = 0;
                    File mostRecentFile = null;
                    for (File f : files) {
                        long lastModified = f.lastModified();
                        if (lastModified > mostRecentLastModified) {
                            mostRecentLastModified = lastModified;
                            mostRecentFile = f;
                        }
                    }

                    if (mostRecentFile != null) {
                        String mimeType = MimeTypesUtil.determineMimeType(mostRecentFile.getName());
                        return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                                mimeType,
                                new FileInputStream(mostRecentFile));
                    }
                }

                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "");
            }
        });

        addAssetWebHandlers(manager, assetManager, "tuning");
    }
}
