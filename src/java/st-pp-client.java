import com.opalkelly.frontpanel.*;
import com.opalkelly.frontpanel.okCFrontPanel.ErrorCode;

import nom.tam.fits.*;

import java.awt.Color;
import javafx.scene.image.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.function.Consumer;

import javafx.application.Application;
import static javafx.application.Application.launch;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.geometry.Insets;
import javafx.geometry.Point2D;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.geometry.Pos;
import javafx.geometry.Rectangle2D;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.Separator;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.CornerRadii;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javax.imageio.ImageIO;

public class OkTst extends Application {

    @Override
    public void start(Stage stage) throws Exception {
        System.loadLibrary("okjFrontPanel");

        EnvController env = new EnvController();

        StackPane root = new StackPane(env.getRoot());

        Scene scene = new Scene(root);
        stage.setTitle("FPGA Interface");
        stage.setScene(scene);
        stage.show();
    }

    /**
     * The main() method is ignored in correctly deployed JavaFX application.
     *
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        launch(args);
    }

}

class ImageWrapper {
    
    Fits fits;
    
    final int IMAGE_BITS = 1036800;
    final int IMAGE_BYTES = 129600;
    final int IMAGE_H = 270;
    final int IMAGE_W = 480;
    final int channel = 0;

    final String file_in = "input";
    final String file_out = "output";
    final String file_in_bmp = "input.bmp";
    //final String file_in_bmp = "test_fits.bmp";
    
    final String file_out_bmp = "output.bmp";

    BooleanProperty loaded = new SimpleBooleanProperty(false);
    BooleanProperty saved = new SimpleBooleanProperty(false);

    private byte[] imageBytes;

    Image image;

    Consumer<String> logger = (logevent) -> {
        System.out.println(logevent);
    };

    public ImageWrapper() {
        reset();
    }

    void setLogger(Consumer<String> logger) {
        this.logger = logger;
    }

    Image getImage() {
        return image;
    }

    int getRGBValue(BufferedImage bi, int x, int y) {
        Color px = new Color(bi.getRGB(x, y));
        return channel == 0 ? px.getRed() : channel == 1 ? px.getGreen() : px.getBlue();
    }

    void setRGBValue(BufferedImage bi, int x, int y, int value) {
        Color px = new Color(value, value, value);
        bi.setRGB(x, y, px.getRGB());
    }

    void loadImage() {
        logger.accept("Loading image " + file_in_bmp);
        logger.accept("Parameters = Resolution: " + IMAGE_H + " x " + IMAGE_W + " | Bit depth: " + 8 + " | Channel " + channel);
        try {
            image = new Image(file_in_bmp);
            BufferedImage imgBuf = ImageIO.read(new File(file_in_bmp));
            int i, j, t = 0;
            for (i = 0; i < IMAGE_W; i++) {
                for (j = 0; j < IMAGE_H; j++) {
                    imageBytes[t++] = (byte) getRGBValue(imgBuf, i, j);
                }
            }

            logger.accept("Image loaded");
            loaded.set(true);
            loadFits();
            
        } catch (IOException ex) {
            logger.accept(ex.toString());
        }
    }

    void loadFits(){
        try {
            fits = new Fits("../data/test.fits");   
            BasicHDU fimg = fits.getHDU(0);
//            fimg.getData();
            ImageData imageData = (ImageData) fimg.getData();
//            int[][] fimage = (int[][]) imageData.getData();
//            Object[] hdu = fits.getHDU(0).getColumns();
//            int[][] fimage = (int[][]) hdu.getKernel();
//            int j;
//            for(int i = 0;i<15;i++) {
//                j = fimage[i][0];
//            }
//            ImageData imageData = (ImageData) hdu.getData();
//            int[][] fimage = (int[][]) imageData.getData();
            
//            logger.accept(image.)
        } catch (Exception e) {
            logger.accept(e.toString());
        }
    }
    
    void saveImage() {
        logger.accept("Writing image " + file_out_bmp);
        try {
            BufferedImage bi = new BufferedImage(IMAGE_W, IMAGE_H, BufferedImage.TYPE_INT_RGB);
            int i, j, t = 0;
            for (i = 0; i < IMAGE_W; i++) {
                for (j = 0; j < IMAGE_H; j++) {
                    setRGBValue(bi, i, j, Byte.toUnsignedInt(imageBytes[t++]));
                }
            }
            ImageIO.write(bi, "bmp", new File(file_out_bmp));
            logger.accept("Writing image done");
            saved.set(true);
        } catch (IOException ex) {
            logger.accept(ex.toString());
        }
    }

    byte[] getBytes() {
        return imageBytes;
    }

    void reset() {
        imageBytes = new byte[IMAGE_BYTES];
    }

}

final class FPGAController {

    okCFrontPanel device = new okCFrontPanel();

    final int block_size = 64;

    private final String configuration_file = "ramtest.bit";

    BooleanProperty deviceOpen = new SimpleBooleanProperty(false);
    BooleanProperty deviceProgrammed = new SimpleBooleanProperty(false);

    Consumer<String> logger = (logevent) -> {
        System.out.println(logevent);
    };

    FPGAController() {
        initFPGA();
    }

    boolean initFPGA() {
        logger.accept("Connecting to device...");
        ErrorCode e1 = device.OpenBySerial();
        device.LoadDefaultPLLConfiguration();
        logger.accept(e1.toString());
        logger.accept(e1 == okCFrontPanel.ErrorCode.NoError ? "Connected!" : "Connection failed");
        deviceOpen.set(e1 == okCFrontPanel.ErrorCode.NoError);
        return e1 == okCFrontPanel.ErrorCode.NoError;
    }

    void closeFPGA() {
        logger.accept("Device closed");
        deviceOpen.set(false);
        deviceProgrammed.set(false);
        device.Close();
    }

    void configure() {
        boolean success = false;
        logger.accept("Programming device with bitstream: " + configuration_file);
        ErrorCode e1 = device.ConfigureFPGA(configuration_file);
        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
            device.ResetFPGA(); // Performs a RESET of the FPGA internals
            return;
        }
        try {
            Thread.sleep(2000);
        } catch (InterruptedException ex) {
            logger.accept(ex.toString());
        }
        device.UpdateWireOuts();
        if ((device.GetWireOutValue(0x20) & 0x01) != 0x01) {
            logger.accept("Memory calibration failed");
        } else {
            logger.accept("Programming done");
            success = true;
        }
        deviceProgrammed.set(success);
    }

    void writeImage(ImageWrapper img) {
        logger.accept("Writing image...");

        // Reset FIFOs
        device.SetWireInValue(0x00, 0x0004);
        device.UpdateWireIns();
        device.SetWireInValue(0x00, 0x0000);
        device.UpdateWireIns();

        // Enable SDRAM read memory transfers
        device.SetWireInValue(0x00, 0x0002);
        device.UpdateWireIns();

        logger.accept("Bytes Written: " + device.WriteToBlockPipeIn(0x80, block_size, img.getBytes().length, img.getBytes()));

        device.UpdateWireOuts();

        logger.accept("Done");
    }

    void readImage(ImageWrapper img) {
        logger.accept("Reading image...");
        // Reset FIFOs
        device.SetWireInValue(0x00, 0x0004);
        device.UpdateWireIns();
        device.SetWireInValue(0x00, 0x0000);
        device.UpdateWireIns();

        // Enable SDRAM read memory transfers
        device.SetWireInValue(0x00, 0x0001);
        device.UpdateWireIns();

        logger.accept("Bytes read: " + device.ReadFromBlockPipeOut(0xA0, block_size, img.getBytes().length, img.getBytes()));
        logger.accept("Done");
    }

    void runProcess() {
        logger.accept("Running process");

        // Enable processing
        device.SetWireInValue(0x00, 0x0008);
        device.UpdateWireIns();

        device.SetWireInValue(0x00, 0x0000);
        device.UpdateWireIns();

    }

    void readMetrics() {
        logger.accept("Reading metrics");
        long[] dataout1 = new long[1];
        long[] dataout2 = new long[1];
        long[] dataout3 = new long[1];
        long[] dataout4 = new long[1];

        ErrorCode e1 = device.ReadRegister(0x00, dataout1);
        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
        }

        e1 = device.ReadRegister(0x01, dataout2);
        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
        }

        e1 = device.ReadRegister(0x02, dataout3);
        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
        }

        e1 = device.ReadRegister(0x03, dataout4);

        if (e1 == ErrorCode.NoError) {
            logger.accept("Metrics = Pipeline total: " + String.format("%.2f", (dataout1[0] / 81250.0)) + "ms | Box filter: " + String.format("%.2f", (dataout2[0] / 81250.0)) + " | CCL: " + String.format("%.2f", (dataout3[0] / 81250.0)) + "ms | Framing: " + String.format("%.2f", (dataout4[0] / 81250.0)) + "ms");
        } else {
            logger.accept(e1.toString());
        }
    }

    void writeSettings(long treshold, long brightness, long timerCount) {

        logger.accept("Writing settings to FPGA");

        ErrorCode e1 = device.WriteRegister(0x0001, treshold);

        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
        }

        e1 = device.WriteRegister(0x0000, brightness);

        if (e1 != ErrorCode.NoError) {
            logger.accept(e1.toString());
        }

        e1 = device.WriteRegister(0x0002, timerCount);

        if (e1 == ErrorCode.NoError) {
            logger.accept("Writing settings done");
        } else {
            logger.accept(e1.toString());
        }
    }

    void setLogger(Consumer<String> logger) {
        this.logger = logger;
    }
}

class EnvController {

    FPGAController fpgaController = new FPGAController();

    StatusView view = new StatusView();

    ImageWrapper inputImageWrapper = new ImageWrapper();
    ImageWrapper outputImageWrapper = new ImageWrapper();

    EnvController() {
        initView();
        fpgaController.setLogger(view.logger);
        inputImageWrapper.setLogger(view.logger);
        outputImageWrapper.setLogger(view.logger);
    }

    private void initView() {
        String status = "";
        status += fpgaController.device.GetDeviceID() + " ";
        status += fpgaController.device.IsOpen() ? "Open" : "Unavailable" + "";

        view.openDeviceButton.setText(fpgaController.device.IsOpen() ? "Close Device" : "Open Device");

        view.statusField.setText(status);

        view.openDeviceButton.setOnAction((event) -> {

            String stat;

            if (fpgaController.device.IsOpen()) {
                fpgaController.closeFPGA();
                view.statusField.setText("Unavailable");
                view.deviceLabel.setText(fpgaController.device.GetDeviceID());
                view.openDeviceButton.setText("Open Device");
            } else {
                if (fpgaController.initFPGA()) {
                    view.statusField.setText("Open");
                    view.openDeviceButton.setText("Close Device");
                } else {
                    stat = view.statusField.getText();
                }
            }
        });

        view.writeConfigurationButton.setOnAction((event) -> {
            fpgaController.configure();
        });

        view.writeImageDataButton.setOnAction((event) -> {
            fpgaController.writeImage(inputImageWrapper);
        });

        view.readImageDataButton.setOnAction((event) -> {
            outputImageWrapper.reset();
            fpgaController.readImage(outputImageWrapper);
            fpgaController.readMetrics();
            outputImageWrapper.saveImage();
            view.outputImage.showImage(outputImageWrapper.file_out_bmp);
        });

        view.loadImageButton.setOnAction((event) -> {
            inputImageWrapper.loadImage();
            view.inputImage.showImage(inputImageWrapper.file_in_bmp);
        });

        view.runButton.setOnAction((event) -> {
            fpgaController.runProcess();
        });

        view.writeSettingsButton.setOnAction((event) -> {
            long th, br, cnt;
            try {
                th = Long.parseLong(view.transformTresholdSettingField.getText());
                br = Long.parseLong(view.brightnessTresholdSettingField.getText());
                cnt = Long.parseLong(view.timerSettingField.getText());
                if (th > 255 | th < 0 | br < 0 | br > Integer.MAX_VALUE | cnt < 0 | cnt > Integer.MAX_VALUE) {
                    throw new NumberFormatException();
                }
            } catch (NumberFormatException e) {
                System.out.println(e);
                th = 1;
                br = 1;
                cnt = 81250000;
            }
            fpgaController.writeSettings(th, br, cnt);
        });

        view.readMetricsButton.setOnAction((event) -> {
            fpgaController.readMetrics();
        });

        view.showInputImage.disableProperty().bind(inputImageWrapper.loaded.not());
        view.writeImageDataButton.disableProperty().bind(inputImageWrapper.loaded.not().or(fpgaController.deviceOpen.not().or(fpgaController.deviceProgrammed.not())));
        view.showOutputImage.disableProperty().bind(outputImageWrapper.saved.not());
        view.readImageDataButton.disableProperty().bind(fpgaController.deviceOpen.not().or(fpgaController.deviceProgrammed.not()));

        view.writeConfigurationButton.disableProperty().bind(fpgaController.deviceOpen.not());
        view.writeSettingsButton.disableProperty().bind(fpgaController.deviceOpen.not().or(fpgaController.deviceProgrammed.not()));
        view.readMetricsButton.disableProperty().bind(fpgaController.deviceOpen.not().or(fpgaController.deviceProgrammed.not()));
        view.runButton.disableProperty().bind(fpgaController.deviceOpen.not().or(fpgaController.deviceProgrammed.not()));

    }

    Node getRoot() {
        return view;
    }

}

class StatusView extends VBox {

    VBox imageConfigBox = new VBox(5);
    VBox statusBox = new VBox(5);
    VBox fpgaConfigBox = new VBox(5);
    VBox imageConfigBody = new VBox(10);
    VBox fpgaConfigBody = new VBox(10);
    VBox statusBody = new VBox(10);
    VBox sysLogBox = new VBox(5);

    HBox buttonBox = new HBox(5);
    HBox bottomBox = new HBox(5);
    HBox topBox = new HBox(5);
    HBox checkBoxBox = new HBox(5);
    HBox systemLogBox = new HBox();
    HBox inputImageBox = new HBox(5);
    HBox outputImageBox = new HBox(5);
    HBox fpgaInitBox = new HBox(5);
    HBox fpgaSettingsBox1 = new HBox(5);
    HBox fpgaSettingsBox2 = new HBox(5);
    HBox fpgaSettingsBox3 = new HBox(5);
    HBox fpgaActionsBox = new HBox(5);

    Label statusLabel = new Label("Device status");
    Label statusField = new Label("");
    Label imageConfigLabel = new Label("Image Settings");
    Label configLabel = new Label("FPGA Settings");
    Label inputImageLabel = new Label("Select input image");
    Label outputImageLabel = new Label("FPGA Read/Write");
    Label settingsLabel = new Label("Settings");
    Label actionsLabel = new Label("Actions");
    Label tresholdLabel = new Label("Treshold for 1-bit transform (0-255)");
    Label brightnessLabel = new Label("Treshold for feature detection (32-bit)");
    Label deviceLabel = new Label("");
    Label timerLabel = new Label("FGPA Timer cycle count");

    Button writeConfigurationButton = new Button("Program device");
    Button writeImageDataButton = new Button("Write imgage");
    Button readImageDataButton = new Button("Read imgage");
    Button writeTresholdRegisterButton = new Button("Write imgage");
    Button openDeviceButton = new Button("Open device");
    Button loadImageButton = new Button("Load image");
    Button readMetricsButton = new Button("Read metrics");
    Button runButton = new Button("Run process");
    Button writeSettingsButton = new Button("Write settings");

    ImageViewer inputImage = new ImageViewer();
    ImageViewer outputImage = new ImageViewer();

    CheckBox showInputImage = new CheckBox("Show input image");
    CheckBox showOutputImage = new CheckBox("Show output Image");
    CheckBox showSysLog = new CheckBox("Show system log");

    TextField transformTresholdSettingField = new TextField("80");
    TextField brightnessTresholdSettingField = new TextField("500");
    TextField timerSettingField = new TextField("81250000");

    SimpleDateFormat formatter_long = new SimpleDateFormat("yyyy-MM-dd 'at' HH:mm:ss");
    SimpleDateFormat formatter_short = new SimpleDateFormat("HH:mm:ss");

    TextArea systemLogArea = new TextArea("------------- System Log ------------ opened:  " + formatter_long.format(new Date(System.currentTimeMillis())) + "\n");

    Consumer<String> logger = (log) -> {
        systemLogArea.appendText("\n[ " + formatter_short.format(new Date(System.currentTimeMillis())) + " ] > " + log);
        systemLogArea.setScrollTop(0);
    };

    StatusView() {
        super(5);
        setPadding(new Insets(20));
        setMinHeight(400);
        setMinWidth(400);
        this.setBackground(new Background(new BackgroundFill(new javafx.scene.paint.Color(0.5, 0.5, 0.5, 1), CornerRadii.EMPTY, Insets.EMPTY)));
        initView();
    }

    private void initView() {

        statusLabel.setFont(Font.font(18));
        statusBox.setPadding(new Insets(10));
        statusField.setMinWidth(100);
        statusBody.getChildren().addAll(deviceLabel, statusField, showSysLog);
        statusField.setAlignment(Pos.CENTER);
        statusBody.setAlignment(Pos.CENTER);
        statusBox.getChildren().addAll(statusLabel, statusBody);
        statusBox.setBackground(new Background(new BackgroundFill(new javafx.scene.paint.Color(1, 1, 1, 0.9), new CornerRadii(5), Insets.EMPTY)));
        statusBox.setAlignment(Pos.TOP_CENTER);

        inputImageBox.getChildren().addAll(loadImageButton, showInputImage);
        outputImageBox.getChildren().addAll(readImageDataButton, showOutputImage);

        imageConfigBody.getChildren().addAll(inputImageLabel, inputImageBox, outputImageLabel, writeImageDataButton, outputImageBox);
        imageConfigBody.setAlignment(Pos.CENTER_LEFT);

        imageConfigLabel.setFont(Font.font(18));
        imageConfigBox.setPadding(new Insets(10));
        imageConfigBox.getChildren().addAll(imageConfigLabel, imageConfigBody);
        imageConfigBox.setBackground(new Background(new BackgroundFill(new javafx.scene.paint.Color(1, 1, 1, 0.9), new CornerRadii(5), Insets.EMPTY)));
        imageConfigBox.setAlignment(Pos.TOP_CENTER);

        fpgaInitBox.getChildren().addAll(openDeviceButton, writeConfigurationButton);
        fpgaActionsBox.getChildren().addAll(runButton, readMetricsButton);

        fpgaSettingsBox1.getChildren().addAll(tresholdLabel, transformTresholdSettingField);
        fpgaSettingsBox1.setAlignment(Pos.CENTER_LEFT);

        fpgaSettingsBox2.getChildren().addAll(brightnessLabel, brightnessTresholdSettingField);
        fpgaSettingsBox2.setAlignment(Pos.CENTER_LEFT);

        fpgaSettingsBox3.getChildren().addAll(timerLabel, timerSettingField);
        fpgaSettingsBox3.setAlignment(Pos.CENTER_LEFT);

        fpgaConfigBody.getChildren().addAll(fpgaInitBox, fpgaSettingsBox1, fpgaSettingsBox2, fpgaSettingsBox3, writeSettingsButton, fpgaActionsBox);
        fpgaConfigBody.setAlignment(Pos.CENTER_LEFT);

        configLabel.setFont(Font.font(18));
        fpgaConfigBox.setPadding(new Insets(10));
        fpgaConfigBox.getChildren().addAll(configLabel, fpgaConfigBody);
        fpgaConfigBox.setBackground(new Background(new BackgroundFill(new javafx.scene.paint.Color(1, 1, 1, 0.9), new CornerRadii(5), Insets.EMPTY)));
        fpgaConfigBox.setAlignment(Pos.TOP_CENTER);

        HBox.setHgrow(statusBox, Priority.ALWAYS);
        HBox.setHgrow(imageConfigBox, Priority.ALWAYS);
        HBox.setHgrow(fpgaConfigBox, Priority.ALWAYS);

        bottomBox.getChildren().addAll(inputImage, outputImage);
        bottomBox.setAlignment(Pos.CENTER);

        topBox.getChildren().addAll(imageConfigBox, statusBox, fpgaConfigBox);

        sysLogBox.getChildren().addAll(systemLogArea);
        sysLogBox.setMinHeight(100);
        sysLogBox.setMaxHeight(100);

        sysLogBox.visibleProperty().bind(showSysLog.selectedProperty());
        sysLogBox.managedProperty().bind(showSysLog.selectedProperty());
        inputImage.visibleProperty().bind(showInputImage.selectedProperty());
        //inputImage.managedProperty().bind(showInputImage.selectedProperty());
        outputImage.visibleProperty().bind(showOutputImage.selectedProperty());
        //outputImage.managedProperty().bind(showOutputImage.selectedProperty());
        bottomBox.managedProperty().bind(showInputImage.selectedProperty().or(showOutputImage.selectedProperty()));

        getChildren().addAll(topBox, sysLogBox, new Separator(), bottomBox);
        setAlignment(Pos.CENTER_LEFT);

    }
}

class ImageViewer extends VBox {

    private static final int MIN_PIXELS = 5;

    void showImage(String IMAGE_URL) {
        Image image = new Image(IMAGE_URL);
        double width = image.getWidth();
        double height = image.getHeight();

        ImageView imageView = new ImageView(image);
        imageView.setSmooth(false);
        imageView.setPreserveRatio(true);
        reset(imageView, width / 2, height / 2);

        ObjectProperty<Point2D> mouseDown = new SimpleObjectProperty<>();

        imageView.setOnMousePressed(e -> {

            Point2D mousePress = imageViewToImage(imageView, new Point2D(e.getX(), e.getY()));
            mouseDown.set(mousePress);
        });

        imageView.setOnMouseDragged(e -> {
            Point2D dragPoint = imageViewToImage(imageView, new Point2D(e.getX(), e.getY()));
            shift(imageView, dragPoint.subtract(mouseDown.get()));
            mouseDown.set(imageViewToImage(imageView, new Point2D(e.getX(), e.getY())));
        });

        imageView.setOnScroll(e -> {
            double delta = e.getDeltaY();
            Rectangle2D viewport = imageView.getViewport();

            double scale = clamp(Math.pow(1.01, delta),
                    // don't scale so we're zoomed in to fewer than MIN_PIXELS in any direction:
                    Math.min(MIN_PIXELS / viewport.getWidth(), MIN_PIXELS / viewport.getHeight()),
                    // don't scale so that we're bigger than image dimensions:
                    Math.max(width / viewport.getWidth(), height / viewport.getHeight())
            );

            Point2D mouse = imageViewToImage(imageView, new Point2D(e.getX(), e.getY()));

            double newWidth = viewport.getWidth() * scale;
            double newHeight = viewport.getHeight() * scale;

            // To keep the visual point under the mouse from moving, we need
            // (x - newViewportMinX) / (x - currentViewportMinX) = scale
            // where x is the mouse X coordinate in the image
            // solving this for newViewportMinX gives
            // newViewportMinX = x - (x - currentViewportMinX) * scale 
            // we then clamp this value so the image never scrolls out
            // of the imageview:
            double newMinX = clamp(mouse.getX() - (mouse.getX() - viewport.getMinX()) * scale,
                    0, width - newWidth);
            double newMinY = clamp(mouse.getY() - (mouse.getY() - viewport.getMinY()) * scale,
                    0, height - newHeight);

            imageView.setViewport(new Rectangle2D(newMinX, newMinY, newWidth, newHeight));
        });

        imageView.setOnMouseClicked(e -> {
            if (e.getClickCount() == 2) {
                reset(imageView, width, height);
            }
        });

        HBox buttons = createButtons(width, height, imageView);
        Tooltip tooltip = new Tooltip("Scroll to zoom, drag to pan");
        Tooltip.install(buttons, tooltip);

        Pane container = new Pane(imageView);
        container.setPrefSize(800, 600);

        imageView.fitWidthProperty().bind(container.widthProperty());
        imageView.fitHeightProperty().bind(container.heightProperty());

        getChildren().setAll(container, buttons);
        setFillWidth(true);
        VBox.setVgrow(this, Priority.ALWAYS);

    }

    private HBox createButtons(double width, double height, ImageView imageView) {
        Button reset = new Button("Reset");
        reset.setOnAction(e -> reset(imageView, width / 2, height / 2));
        Button full = new Button("Full view");
        full.setOnAction(e -> reset(imageView, width, height));
        HBox buttons = new HBox(10, reset, full);
        buttons.setAlignment(Pos.TOP_LEFT);
        buttons.setPadding(new Insets(10));
        return buttons;
    }

    // reset to the top left:
    private void reset(ImageView imageView, double width, double height) {
        imageView.setViewport(new Rectangle2D(0, 0, width, height));
    }

    // shift the viewport of the imageView by the specified delta, clamping so
    // the viewport does not move off the actual image:
    private void shift(ImageView imageView, Point2D delta) {
        Rectangle2D viewport = imageView.getViewport();

        double width = imageView.getImage().getWidth();
        double height = imageView.getImage().getHeight();

        double maxX = width - viewport.getWidth();
        double maxY = height - viewport.getHeight();

        double minX = clamp(viewport.getMinX() - delta.getX(), 0, maxX);
        double minY = clamp(viewport.getMinY() - delta.getY(), 0, maxY);

        imageView.setViewport(new Rectangle2D(minX, minY, viewport.getWidth(), viewport.getHeight()));
    }

    private double clamp(double value, double min, double max) {

        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    // convert mouse coordinates in the imageView to coordinates in the actual image:
    private Point2D imageViewToImage(ImageView imageView, Point2D imageViewCoordinates) {
        double xProportion = imageViewCoordinates.getX() / imageView.getBoundsInLocal().getWidth();
        double yProportion = imageViewCoordinates.getY() / imageView.getBoundsInLocal().getHeight();

        Rectangle2D viewport = imageView.getViewport();
        return new Point2D(
                viewport.getMinX() + xProportion * viewport.getWidth(),
                viewport.getMinY() + yProportion * viewport.getHeight());
    }

}
