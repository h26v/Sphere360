package com.sphere360.app.stitch;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Chép file JPEG kèm khối XMP GPano để Google Photos, Facebook và các trình xem 360 nhận ra ảnh cầu.
 */
final class GPanoWriter {

    private static final byte[] XMP_HEADER = "http://ns.adobe.com/xap/1.0/\0".getBytes(StandardCharsets.US_ASCII);

    private GPanoWriter() {}

    static void copyWithXmp(File jpeg, int width, int height, int sourceCount, OutputStream out) throws IOException {
        byte[] xmp = buildXmp(width, height, sourceCount).getBytes(StandardCharsets.UTF_8);
        try (DataInputStream in = new DataInputStream(new BufferedInputStream(new FileInputStream(jpeg), 1 << 16))) {
            if (in.readUnsignedShort() != 0xFFD8) throw new IOException("Không phải file JPEG");
            out.write(0xFF);
            out.write(0xD8);

            // Giữ APP0 (JFIF) đứng đầu nếu có, rồi chèn APP1 XMP ngay sau.
            in.mark(4);
            int marker = in.readUnsignedShort();
            if (marker == 0xFFE0) {
                int len = in.readUnsignedShort();
                byte[] seg = new byte[len - 2];
                in.readFully(seg);
                out.write(0xFF);
                out.write(0xE0);
                out.write(len >> 8);
                out.write(len & 0xFF);
                out.write(seg);
            } else {
                in.reset();
            }

            int len = 2 + XMP_HEADER.length + xmp.length;
            if (len > 0xFFFF) throw new IOException("XMP quá lớn");
            out.write(0xFF);
            out.write(0xE1);
            out.write(len >> 8);
            out.write(len & 0xFF);
            out.write(XMP_HEADER);
            out.write(xmp);

            byte[] buf = new byte[1 << 16];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        }
    }

    private static String buildXmp(int w, int h, int sourceCount) {
        return String.format(Locale.US,
                "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"Sphere360\">" +
                        "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">" +
                        "<rdf:Description rdf:about=\"\" xmlns:GPano=\"http://ns.google.com/photos/1.0/panorama/\"" +
                        " GPano:ProjectionType=\"equirectangular\"" +
                        " GPano:UsePanoramaViewer=\"True\"" +
                        " GPano:CroppedAreaImageWidthPixels=\"%1$d\"" +
                        " GPano:CroppedAreaImageHeightPixels=\"%2$d\"" +
                        " GPano:FullPanoWidthPixels=\"%1$d\"" +
                        " GPano:FullPanoHeightPixels=\"%2$d\"" +
                        " GPano:CroppedAreaLeftPixels=\"0\"" +
                        " GPano:CroppedAreaTopPixels=\"0\"" +
                        " GPano:PoseHeadingDegrees=\"0\"" +
                        " GPano:SourcePhotosCount=\"%3$d\"" +
                        " GPano:CaptureSoftware=\"Sphere360\"" +
                        " GPano:StitchingSoftware=\"Sphere360 (OpenCV)\"/>" +
                        "</rdf:RDF></x:xmpmeta>",
                w, h, sourceCount);
    }
}
