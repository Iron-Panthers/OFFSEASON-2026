package frc.robot.utility.rendering;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.Iterator;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.imageio.stream.MemoryCacheImageOutputStream;

/**
 * Serves one camera as motion JPEG over localhost, the way PhotonVision does.
 *
 * <p>Presenting the renderer as an ordinary MJPEG endpoint rather than as a Java API is the whole
 * point: anything that can already consume a PhotonVision stream, an OpenCV script, a training data
 * collector, a browser, works against this with no changes at all.
 *
 * <p>Built on the JDK HTTP server so the robot project picks up no new dependency for it.
 *
 * <p>Endpoints:
 *
 * <ul>
 *   <li>{@code /stream.mjpg} the live stream
 *   <li>{@code /snapshot.jpg} the most recent frame, once
 *   <li>{@code /info.json} resolution, frame count and viewer count
 *   <li>{@code /} a page with the stream embedded, for opening in a browser
 * </ul>
 */
final class MjpegServer {

  private static final String BOUNDARY = "frameboundary";

  /** Long enough that an idle stream still sends something before a proxy gives up on it. */
  private static final long FRAME_WAIT_MILLIS = 2000;

  private final String name;
  private final int port;
  private final HttpServer server;

  /** Number of clients currently reading the stream, used to skip rendering nobody is watching. */
  private final AtomicInteger viewers = new AtomicInteger();

  /**
   * When a still was last asked for.
   *
   * <p>A snapshot request has to count as demand in its own right. Rendering only for stream
   * clients means a script that just wants one frame asks for one, gets nothing because nothing has
   * rendered, and nothing ever will.
   *
   * <p>Epoch rather than {@link Long#MIN_VALUE} as the "never asked" value. Subtracting MIN_VALUE
   * from the current time overflows, the difference comes out negative, and every camera then looks
   * permanently in demand, which quietly defeats the whole point of gating on it.
   */
  private final AtomicLong lastSnapshotRequest = new AtomicLong(0);

  /** How long a single snapshot request keeps the camera rendering. */
  private static final long SNAPSHOT_DEMAND_MILLIS = 5000;

  /** How long a snapshot request waits for the first frame before giving up. */
  private static final long SNAPSHOT_TIMEOUT_MILLIS = 20000;

  private final Object frameLock = new Object();
  private byte[] latestFrame;
  private long frameCounter;
  private volatile int width;
  private volatile int height;

  private MjpegServer(String name, int port, HttpServer server) {
    this.name = name;
    this.port = port;
    this.server = server;
  }

  /**
   * Starts a server for one camera.
   *
   * @param port bound on the loopback interface only; this is simulation output, not something to
   *     expose to the network
   */
  static MjpegServer start(String name, int port) throws IOException {
    HttpServer http = HttpServer.create(new InetSocketAddress("127.0.0.1", port), 4);
    MjpegServer served = new MjpegServer(name, port, http);

    // Streaming handlers hold their thread for as long as the client watches, so the pool has to
    // grow rather than queue.
    http.setExecutor(
        Executors.newCachedThreadPool(
            runnable -> {
              Thread thread = new Thread(runnable, "RenderStream-" + port);
              thread.setDaemon(true);
              return thread;
            }));

    http.createContext("/stream.mjpg", served::handleStream);
    http.createContext("/snapshot.jpg", served::handleSnapshot);
    http.createContext("/info.json", served::handleInfo);
    http.createContext("/", served::handleIndex);
    http.start();
    return served;
  }

  int port() {
    return port;
  }

  String name() {
    return name;
  }

  /**
   * @return true if anything is waiting on frames from this camera, either streaming or having
   *     recently asked for a still
   */
  boolean isWanted() {
    return viewers.get() > 0
        || System.currentTimeMillis() - lastSnapshotRequest.get() < SNAPSHOT_DEMAND_MILLIS;
  }

  /** Publishes a frame, waking every connected client. */
  void publish(BufferedImage image, float quality) throws IOException {
    byte[] encoded = encodeJpeg(image, quality);
    width = image.getWidth();
    height = image.getHeight();
    synchronized (frameLock) {
      latestFrame = encoded;
      frameCounter++;
      frameLock.notifyAll();
    }
  }

  void stop() {
    synchronized (frameLock) {
      // Release anything blocked waiting for a frame that is never coming.
      frameLock.notifyAll();
    }
    server.stop(0);
  }

  private void handleStream(HttpExchange exchange) throws IOException {
    if (!"GET".equals(exchange.getRequestMethod())) {
      exchange.sendResponseHeaders(405, -1);
      return;
    }
    exchange
        .getResponseHeaders()
        .set("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY);
    exchange.getResponseHeaders().set("Cache-Control", "no-store");
    exchange.sendResponseHeaders(200, 0);

    viewers.incrementAndGet();
    long sent = -1;
    try (OutputStream out = exchange.getResponseBody()) {
      while (true) {
        byte[] frame;
        synchronized (frameLock) {
          // Wait for something newer, but give up waiting periodically and resend the current
          // frame: a browser drops a multipart stream that goes completely silent, and the
          // renderer can easily take a second or two between frames.
          while (latestFrame == null || frameCounter == sent) {
            try {
              frameLock.wait(FRAME_WAIT_MILLIS);
            } catch (InterruptedException interrupted) {
              Thread.currentThread().interrupt();
              return;
            }
            if (latestFrame != null) {
              break;
            }
          }
          frame = latestFrame;
          sent = frameCounter;
        }

        out.write(("--" + BOUNDARY + "\r\n").getBytes(StandardCharsets.US_ASCII));
        out.write("Content-Type: image/jpeg\r\n".getBytes(StandardCharsets.US_ASCII));
        out.write(
            ("Content-Length: " + frame.length + "\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
        out.write(frame);
        out.write("\r\n".getBytes(StandardCharsets.US_ASCII));
        out.flush();
      }
    } catch (IOException clientWentAway) {
      // A browser closing its tab is the normal way this ends, not an error worth reporting.
    } finally {
      viewers.decrementAndGet();
      exchange.close();
    }
  }

  private void handleSnapshot(HttpExchange exchange) throws IOException {
    lastSnapshotRequest.set(System.currentTimeMillis());

    byte[] frame;
    synchronized (frameLock) {
      // The request itself is what makes the camera render, so wait for the frame it asked for
      // rather than reporting that none exists yet.
      long deadline = System.currentTimeMillis() + SNAPSHOT_TIMEOUT_MILLIS;
      while (latestFrame == null) {
        long remaining = deadline - System.currentTimeMillis();
        if (remaining <= 0) {
          break;
        }
        try {
          frameLock.wait(remaining);
        } catch (InterruptedException interrupted) {
          Thread.currentThread().interrupt();
          break;
        }
      }
      frame = latestFrame;
    }
    if (frame == null) {
      exchange.sendResponseHeaders(503, -1);
      exchange.close();
      return;
    }
    exchange.getResponseHeaders().set("Content-Type", "image/jpeg");
    exchange.getResponseHeaders().set("Cache-Control", "no-store");
    exchange.sendResponseHeaders(200, frame.length);
    try (OutputStream out = exchange.getResponseBody()) {
      out.write(frame);
    }
    exchange.close();
  }

  private void handleInfo(HttpExchange exchange) throws IOException {
    long frames;
    synchronized (frameLock) {
      frames = frameCounter;
    }
    String json =
        String.format(
            "{\"name\":\"%s\",\"port\":%d,\"width\":%d,\"height\":%d,\"frames\":%d,\"viewers\":%d}",
            name, port, width, height, frames, viewers.get());
    respondText(exchange, "application/json", json);
  }

  private void handleIndex(HttpExchange exchange) throws IOException {
    if (!"/".equals(exchange.getRequestURI().getPath())) {
      exchange.sendResponseHeaders(404, -1);
      exchange.close();
      return;
    }
    String page =
        """
        <!doctype html>
        <title>%s</title>
        <style>
          body { background:#111; color:#ddd; font:14px system-ui, sans-serif; margin:0;
                 display:flex; flex-direction:column; align-items:center; gap:12px; padding:16px; }
          img  { max-width:100%%; image-rendering:pixelated; border:1px solid #333; }
          code { color:#8cf; }
        </style>
        <h1>%s</h1>
        <img src="/stream.mjpg" alt="camera stream">
        <p>Stream <code>/stream.mjpg</code> &middot; still <code>/snapshot.jpg</code>
           &middot; status <code>/info.json</code></p>
        """
            .formatted(name, name);
    respondText(exchange, "text/html; charset=utf-8", page);
  }

  private static void respondText(HttpExchange exchange, String contentType, String body)
      throws IOException {
    byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", contentType);
    exchange.sendResponseHeaders(200, bytes.length);
    try (OutputStream out = exchange.getResponseBody()) {
      out.write(bytes);
    }
    exchange.close();
  }

  /**
   * Encodes at a controlled quality.
   *
   * <p>Quality is a deliberate knob, not a detail. The camera on the robot delivers JPEG, so its
   * compression artefacts are part of what any pipeline has to cope with, and a pipeline validated
   * on lossless frames has not been validated on what it will actually receive.
   */
  private static byte[] encodeJpeg(BufferedImage image, float quality) throws IOException {
    Iterator<ImageWriter> writers = ImageIO.getImageWritersByFormatName("jpeg");
    if (!writers.hasNext()) {
      throw new IOException("No JPEG encoder available in this JVM");
    }
    ImageWriter writer = writers.next();
    try {
      ImageWriteParam params = writer.getDefaultWriteParam();
      params.setCompressionMode(ImageWriteParam.MODE_EXPLICIT);
      params.setCompressionQuality(Math.min(1f, Math.max(0.05f, quality)));

      ByteArrayOutputStream bytes = new ByteArrayOutputStream(1 << 16);
      try (MemoryCacheImageOutputStream stream = new MemoryCacheImageOutputStream(bytes)) {
        writer.setOutput(stream);
        writer.write(null, new IIOImage(image, null, null), params);
      }
      return bytes.toByteArray();
    } finally {
      writer.dispose();
    }
  }
}
