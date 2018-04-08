/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/primo_dash/main_window.hpp"
#include <QProcess>
#include <unistd.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace primo_dash {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //ui.frameMain->setEnabled(false);

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    
    QObject::connect(&qnode, SIGNAL(battUpdate(double, double)), this, SLOT(updateBattView(double, double)));

    // Timer
    startTimer(100);

    curvBattV = new QwtPlotCurve("Battery voltage");


    battVsamples = new QVector<QPointF>;
    curvBattV->attach(ui.plot_batt);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
            ui.frameMain->setEnabled(true);
        }
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
            ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.frameMain->setEnabled(true);
        }
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}


void MainWindow::updateBattView(double bVoltage, double bCurrent)
{
    ui.batt_indVoltage->setValue(bVoltage);
    ui.batt_indCurrent->setValue(bCurrent);

    QwtPointSeriesData* myData = new QwtPointSeriesData;
    battVsamples->push_back(QPointF(battVsamples->size(),bVoltage));
    myData->setSamples(*battVsamples);
    curvBattV->setData(myData);

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "primo_dash");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "primo_dash");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_pB_s2_disableXinput_clicked()
{
    // Launch the process and dump its output to the log window
    QProcess process;
    process.start(QDir::homePath()+"/primo_ws/src/primo_stereo/scripts/stereo2_fix.sh",QIODevice::ReadOnly);
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    qnode.log(QNode::Info, output.toUtf8().constData());
}

}  // namespace primo_dash

void primo_dash::MainWindow::on_pB_s2_viewLeft_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo2/left/image_raw");

    qnode.log(QNode::Info, "Launching left viewer");
}

void primo_dash::MainWindow::on_pB_s2_viewRight_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo2/right/image_raw");

    qnode.log(QNode::Info, "Launching right viewer");
}

void primo_dash::MainWindow::on_pB_robonet_vpn_primo_clicked()  	{ launchRobonet("-n VPN -r primo"); }
void primo_dash::MainWindow::on_pB_robonet_shop_primo_clicked() 	{ launchRobonet("-n shop -r primo"); }
void primo_dash::MainWindow::on_pB_robonet_field_primo_clicked() 	{ launchRobonet("-n field -r primo"); }
void primo_dash::MainWindow::on_pB_robonet_vpn_laptop_clicked() 	{ launchRobonet("-n VPN -r dev-laptop"); }
void primo_dash::MainWindow::on_pB_robonet_shop_laptop_clicked() 	{ launchRobonet("-n shop -r dev-laptop"); }
void primo_dash::MainWindow::on_pB_robonet_field_laptop_clicked() 	{ launchRobonet("-n field -r dev-laptop"); }

void primo_dash::MainWindow::launchRobonet(QString robonetArgs)
{
    // Launch the process and dump its output to the log window
    // Robotnet is added to the sudoers file which is why we can run it as root without asking for a password
    QProcess process;
    process.start("sudo "+QDir::homePath()+"/primo_ws/src/primo_base/scripts/robonet.py " + robonetArgs,QIODevice::ReadOnly);
    process.waitForFinished();
    QString output = process.readAllStandardOutput();

    QMessageBox::information(
        this,
        tr("Robonet info"),
        output);
}

void primo_dash::MainWindow::on_pB_roscore_clicked()
{
    // Launch roscore in the run window of tmux.
    QProcess process;
    process.startDetached("tmux new-session -d -s \"roscore\" \"roscore\"" );
}

void primo_dash::MainWindow::on_pB_rviz_clicked()
{
    // Just launch RVIZ
    QProcess process;
    process.startDetached("roslaunch primo_bringup rviz.launch");
    qnode.log(QNode::Info, "Launching RVIZ");
}

void primo_dash::MainWindow::on_pB_clearCostmap_clicked()
{
    // Launch the process and dump its output to the log window
    QProcess process;
    QString output = "Selecting window: " + process.readAllStandardOutput();
    QString commandStr =
            "tmux new-session -d -s \"costmap\" \"watch -n " +
            QString::number(ui.num_clear_costmap_wait->value()) +
            " rosservice call /move_base/clear_costmaps \"{}\"\"";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();

    output = output + "\nLaunched watch: " + process.readAllStandardOutput();
    qnode.log(QNode::Info, output.toUtf8().constData());
}

void primo_dash::MainWindow::on_pB_cpMap_clicked()
{
    // Here we copy the rtabmap db to a file that we want
    QString fileName = QFileDialog::getSaveFileName(this,
            tr("Select a filename to save the map"), QDir::homePath()+"/new.db",
            tr("rtabmap db (*.db);;All Files (*)"));
    if (fileName.isEmpty())
           return;
    else {
        if(QFile::copy(QDir::homePath()+"/.ros/rtabmap.db",fileName)) {
            QMessageBox::information( this,
                tr("System info"),
                QString("Successfully copied the db to "+fileName));
            qnode.log(QNode::Info, "Copied rtabmap.db successfully");
        }
        else {
            QMessageBox::information( this,
                tr("System info"),
                QString("Failed to copy the db to "+fileName+"\n File has to be new."));
            qnode.log(QNode::Info, "Failed to copy rtabmap.db");
        }
    }
}

void primo_dash::MainWindow::on_pB_qtcam_clicked()
{
    // Just launch qtcam
    QProcess process;
    process.startDetached("qtcam");
}

void primo_dash::MainWindow::on_pB_killQtcam_clicked()
{
    // kill qtcam
    QProcess process;
    process.startDetached("pkill qtcam");
}

// =================================================
void primo_dash::MainWindow::on_s2le_slider_sliderMoved(int position) {
    qnode.pubCamSetting(QNode::s2_left_expo, (double)position);
    ui.s2le_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s2le_spinBox_editingFinished() {
    qnode.pubCamSetting(QNode::s2_left_expo, ui.s2le_spinBox->value());
    ui.s2le_slider->setValue(ui.s2le_spinBox->value());
}

// =================================================
void primo_dash::MainWindow::on_s2lb_slider_sliderMoved(int position) {
    qnode.pubCamSetting(QNode::s2_left_bright, (double)position);
    ui.s2lb_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s2lb_spinBox_editingFinished() {
    qnode.pubCamSetting(QNode::s2_left_bright, ui.s2lb_spinBox->value());
    ui.s2lb_slider->setValue(ui.s2lb_spinBox->value());
}

// =================================================
void primo_dash::MainWindow::on_s2re_slider_sliderMoved(int position)
{
    qnode.pubCamSetting(QNode::s2_right_expo, (double)position);
    ui.s2re_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s2re_spinBox_editingFinished()
{
    qnode.pubCamSetting(QNode::s2_right_expo, ui.s2re_spinBox->value());
    ui.s2re_slider->setValue(ui.s2re_spinBox->value());
}

// =================================================
void primo_dash::MainWindow::on_s2rb_slider_sliderMoved(int position) {
    qnode.pubCamSetting(QNode::s2_right_bright, (double)position);
    ui.s2rb_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s2rb_spinBox_editingFinished() {
    qnode.pubCamSetting(QNode::s2_right_bright, ui.s2rb_spinBox->value());
    ui.s2rb_slider->setValue(ui.s2rb_spinBox->value());
}




// =================================================
void primo_dash::MainWindow::on_s0e_slider_sliderMoved(int position)
{
    qnode.pubCamSetting(QNode::s0_expo, (double)position);
    ui.s0e_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s0e_spinBox_editingFinished()
{
    qnode.pubCamSetting(QNode::s0_expo, ui.s0e_spinBox->value());
    ui.s0e_slider->setValue(ui.s0e_spinBox->value());
}


// =================================================
void primo_dash::MainWindow::on_s0b_slider_sliderMoved(int position)
{
    qnode.pubCamSetting(QNode::s0_bright, (double)position);
    ui.s0b_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s0b_spinBox_editingFinished()
{
    qnode.pubCamSetting(QNode::s0_bright, ui.s0b_spinBox->value());
    ui.s0b_slider->setValue(ui.s0b_spinBox->value());
}

// =================================================
void primo_dash::MainWindow::on_s1e_slider_sliderMoved(int position)
{
    qnode.pubCamSetting(QNode::s1_expo, (double)position);
    ui.s1e_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s1e_spinBox_editingFinished()
{
    qnode.pubCamSetting(QNode::s1_expo, ui.s1e_spinBox->value());
    ui.s1e_slider->setValue(ui.s1e_spinBox->value());
}


// =================================================
void primo_dash::MainWindow::on_s1b_slider_sliderMoved(int position)
{
    qnode.pubCamSetting(QNode::s1_bright, (double)position);
    ui.s1b_spinBox->setValue(position);
}

void primo_dash::MainWindow::on_s1b_spinBox_editingFinished()
{
    qnode.pubCamSetting(QNode::s1_bright, ui.s1b_spinBox->value());
    ui.s1b_slider->setValue(ui.s1b_spinBox->value());
}


void primo_dash::MainWindow::on_pb_launch_base_alpha_clicked()
{
    // Launch roscore in the run window of tmux.
    QProcess process;
    QString cmd = "tmux new-session -d -s \"base_alpha\" \"roslaunch primo_bringup base_alpha.launch";
    if(ui.rB_jsp_gui->isChecked())
            cmd = cmd + " use_gui:=true\"";
    else
        cmd = cmd + "\"";

    process.startDetached(cmd);
}

void primo_dash::MainWindow::on_pushButton_4_clicked()
{
    // Launch roscore in the run window of tmux.
    QProcess process;
    process.startDetached("tmux new-session -d -s \"stereo_bare\" \"roslaunch primo_bringup stereo_bare.launch\"");
}

void primo_dash::MainWindow::on_pB_stereo_suite_clicked()
{
    // Launch roscore in the run window of tmux.
    QProcess process;
    process.startDetached("tmux new-session -d -s \"stereo_suite\" \"roslaunch primo_stereo stereo_suite.launch\"" );
}

void primo_dash::MainWindow::on_pB_killCostmap_clicked()
{
    QProcess process;
    QString command = "pkill watch -n";
    process.startDetached(command);
}

void primo_dash::MainWindow::on_pB_qtcam_2_clicked()
{
    QProcess process;
    process.startDetached("guvcview");
}

void primo_dash::MainWindow::on_pB_qtcam_3_clicked()
{
    QProcess process;
    process.startDetached("pkill guvcview");
}

void primo_dash::MainWindow::on_pB_killRos_clicked()
{
    QProcess process;
    process.startDetached("pkill ros");
}

void primo_dash::MainWindow::on_pB_color0_clicked()
{
    QProcess process;
    process.startDetached("tmux new-session -d -s \"color0\" \"roslaunch primo_base color0.launch\"" );
}

void primo_dash::MainWindow::on_pB_view_s1_right_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo1/right/image_raw");
}

void primo_dash::MainWindow::on_pB_view_s1_left_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo1/left/image_raw");
}

void primo_dash::MainWindow::on_pB_view_s0_right_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo0/right/image_raw");
}

void primo_dash::MainWindow::on_pB_view_s0_left_clicked()
{
    QProcess process;
    process.startDetached("rqt_image_view /stereo0/left/image_raw");
}

void primo_dash::MainWindow::on_pb_kill_rqt_view_clicked()
{
    // Match with pgrep also the arguments
    QProcess process;
    process.startDetached("pkill -f rqt_image_view");
}

void primo_dash::MainWindow::on_pB_kill_rqt_view2_clicked()
{
    // Match with pgrep also the arguments
    QProcess process;
    process.startDetached("pkill -f rqt_image_view");
}

void primo_dash::MainWindow::on_pB_localization_clicked()
{
    QString command = "roslaunch primo_bringup localization.launch "
                      "del_db:=false localize:=true maxError:=" +
                      QString::number(ui.dSb_maxError->value());

    // Make a confirm dialog
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Test", "The launch string is as follows: \n"
                                  + command,
                                    QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        QProcess process;
        QString fullCmd = "tmux new-session -d -s \"map\" " + command;
        process.startDetached(fullCmd);
    }
}

void primo_dash::MainWindow::on_pB_mapping_clicked()
{
    QString command = "roslaunch primo_bringup localization.launch "
                      "del_db:=true localize:=false maxError:=" +
                      QString::number(ui.dSb_maxError->value()) +
                      " rangeMax:=" +
                      QString::number(ui.dSb_maxDepth->value());

    // Make a confirm dialog
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Mapping confirm", " The current map will be deleted!\n "
                                                           "The launch string is as follows: \n"
                                  + command,
                                    QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        QProcess process;
        QString fullCmd = "tmux new-session -d -s \"map\" " + command;
        process.startDetached(fullCmd);
    }
}

void primo_dash::MainWindow::on_pB_kill_base_alpha_clicked()
{
    // Launch the process and dump its output to the log window
    QProcess process;
    QString commandStr = "tmux send -t base_alpha:0.0 C-c ENTER";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();
}

void primo_dash::MainWindow::on_pB_kill_stereo_bare_clicked()
{
    // Launch the process and dump its output to the log window
    QProcess process;
    QString commandStr = "tmux send -t stereo_bare:0.0 C-c ENTER";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();
}

void primo_dash::MainWindow::on_pB_kill_stereo_suite_clicked()
{
    // Launch the process and dump its output to the log window
    QProcess process;
    QString commandStr = "tmux send -t stereo_suite:0.0 C-c ENTER";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();
}

void primo_dash::MainWindow::on_pB_cal_color0_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_base color0_calibrate.launch");
}

void primo_dash::MainWindow::on_pB_driver_color0_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_base color0.launch");
}

void primo_dash::MainWindow::on_pB_yolo2_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_base yolo2_proc.launch");
}

void primo_dash::MainWindow::on_pB_kill_yolo2_clicked()
{
    // Match with pgrep also the arguments
    QProcess process;
    process.startDetached("pkill -f yolo");
}

void primo_dash::MainWindow::on_pB_launch_nav_clicked()
{
    // Here we launch the primo_nav file. If lanes is checked, prompt the user to select a lanes file.
    QProcess process;
    QString cmd = "tmux new-session -d -s \"nav\" \"roslaunch primo_nav primo_nav.launch";

    // Check if lanes is clicked. If it is ask for a path
    if(ui.rB_lanes->isChecked())
    {
        QString fileName = QFileDialog::getOpenFileName(this, ("Open File"),
                                                        QDir::homePath() + "/primo_ws/src/primo_nav/lanes/",
                                                        ("Maps (*.yaml )"));
        cmd = cmd + " lanes:=true lanes_file:=" + fileName +"\"";

        // Check for the user pressing cancel
        if(fileName.isNull())
            return;

        // Everything looks good, so start the nav file
        process.startDetached(cmd);
    }
    // User did not select lanes, which is the default nav launch mode
    else
    {
        cmd = cmd + "\"";
        process.startDetached(cmd);
    }

}

void primo_dash::MainWindow::on_pB_kill_nav_clicked()
{
    QProcess process;
    QString commandStr = "tmux send -t nav:0.0 C-c ENTER";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();
}

void primo_dash::MainWindow::on_pB_align_clicked()
{
    QProcess process;

    // Launch roscore in the run window of tmux.
    QString cmd = "roslaunch primo_description primo_urdf_real.launch ";
    // Start a boolean for using the gui. If we are aligning anything use it.
    bool use_gui = false;

    if(ui.rB_as0->isChecked()) {
        use_gui = true;
        cmd+= " align_s0:=true";
    }

    if(ui.rB_as1->isChecked()) {
        use_gui = true;
        cmd+= " align_s1:=true";
    }

    if(ui.rB_as2->isChecked()) {
        use_gui = true;
        cmd+= " align_s2:=true";
    }

    if(ui.rB_as3->isChecked()) {
        use_gui = true;
        cmd+= " align_s3:=true";
    }

    if(ui.rB_ac0->isChecked()){
        use_gui = true;
        cmd+= " align_c0:=true";
    }

    if(use_gui)
        cmd+= " use_gui:=true";

    process.startDetached(cmd);
}

void primo_dash::MainWindow::on_pB_align_normal_clicked()
{
    QProcess process;
    QString cmd = "roslaunch primo_description primo_urdf_real.launch";
    process.startDetached(cmd);
}

void primo_dash::MainWindow::on_pB_align_save_clicked()
{
    // Run the save script
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QProcess process;
    QString cmd = "rosrun primo_description align_save.py";
    process.start(cmd,QIODevice::ReadOnly);
    process.waitForFinished();

    // Output the results
    QApplication::restoreOverrideCursor();

    QString output = process.readAllStandardOutput();
    output = output + process.readAllStandardError();
    QMessageBox msgBox;
    msgBox.setText(output);
    msgBox.exec();
}

void primo_dash::MainWindow::on_pB_cal_s0_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_stereo calibrate_stereo0.launch");
}

void primo_dash::MainWindow::on_pB_cal_s1_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_stereo calibrate_stereo1.launch");
}

void primo_dash::MainWindow::on_pB_cal_s2_clicked()
{
    QProcess process;
    process.startDetached("roslaunch primo_stereo calibrate_stereo2.launch");
}

void primo_dash::MainWindow::on_pB_cal_copy_s0_clicked()
{
    QString filedate = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QFile::copy("/tmp/calibrationdata.tar.gz", QDir::homePath()+"/primo_ws/src/primo_stereo/config/stereo0/"+filedate+".tar.gz");
}

void primo_dash::MainWindow::on_pB_cal_copy_s1_clicked()
{
    QString filedate = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QFile::copy("/tmp/calibrationdata.tar.gz", QDir::homePath()+"/primo_ws/src/primo_stereo/config/stereo1/"+filedate+".tar.gz");
}

void primo_dash::MainWindow::on_pB_cal_copy_s2_clicked()
{
    QString filedate = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QFile::copy("/tmp/calibrationdata.tar.gz", QDir::homePath()+"/primo_ws/src/primo_stereo/config/stereo2/"+filedate+".tar.gz");
}

void primo_dash::MainWindow::on_pB_cal_kill_clicked()
{
    // Match with pgrep also the arguments
    QProcess process;
    process.startDetached("pkill cameracalibrato");
}

/**
 * @brief Download the current /grid_map and save it in the lanes folder with
 * the given name. Then edit the .yaml and replace the last 3 lines with the new
 * scale and type
 */
void primo_dash::MainWindow::on_pB_save_map_clicked()
{
    // Get the path and name to save
    bool ok;      

    QString mapName = QInputDialog::getText(  this,  tr("Map name"),  
            tr("Enter name of map we will save as. Will be saved in the lanes folder."),   
            QLineEdit::Normal, "", &ok );  

    if( ok && !mapName.isEmpty() )  
    {
        // Create the filepath without extension
        QString file = QDir::homePath() + "/primo_ws/src/primo_nav/lanes/" + mapName;

        // Build the command
        QString cmd = "rosrun map_server map_saver -f " + file + " map:=/grid_map";

        // Set the cursor to loading
        QApplication::setOverrideCursor(Qt::WaitCursor);

        // QProcess process;
        // process.startDetached("tmux new-session -d -s \"mapsaver\" \"" + cmd + "\"");

        // Run the map save script
        QProcess process;
        process.start(cmd,QIODevice::ReadOnly);
        process.waitForFinished();
        QApplication::restoreOverrideCursor();

        QString output = process.readAllStandardOutput();
        output += process.readAllStandardError();

        // Set the thresholds and add the mode: scale
        cmd = R"(sed -i "/occupied_thresh:/c\occupied_thresh: 0.80" )" + file + ".yaml";
        process.start(cmd,QIODevice::ReadOnly);
        process.waitForFinished();
        output += process.readAllStandardOutput();
        output += process.readAllStandardError();

        cmd = R"(sed -i "/free_thresh:/c\free_thresh: 0.20\nmode: scale" )" + file + ".yaml";
        process.start(cmd,QIODevice::ReadOnly);
        process.waitForFinished();
        output += process.readAllStandardOutput();
        output += process.readAllStandardError();

        // Restore the cursor
        QApplication::restoreOverrideCursor();

        // Show us a box of what just happened
        QMessageBox msgBox;
        msgBox.setText(output);
        msgBox.exec();
    }
}

/**
 * @brief Opens a file dialog and allows the user to select a pgm to edit in
 * gimp
 */
void primo_dash::MainWindow::on_pB_edit_map_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, ("Open File"), 
            QDir::homePath() + "/primo_ws/src/primo_nav/lanes/", 
            ("Maps (*.pgm )"));

    // Break out if canceled
    if(fileName.isNull())
        return;

    // Launch gimp and edit the map
    QProcess process;
    process.startDetached( "gimp " + fileName);
}

// ======= Trig ====================================
void primo_dash::MainWindow::on_slider_trig_valueChanged(double value)
{
    qnode.pubCamSetting(QNode::s2_trig, value);
    ui.s2_trig_spinBox->setValue(value);
}

void primo_dash::MainWindow::on_pB_s2_expoRang_clicked()
{
    bool ok;
    int range = QInputDialog::getInt(this, tr("S2 max expo range"),
                                     tr("Enter exposure range. 200 is maximum for outside, 1000 max overall"),1000,1,1000,1,&ok);
    if (ok)
    {
        ui.s2le_slider->setRange(1,range);
        ui.s2re_slider->setRange(1,range);
    }
}

// Timer event!
void primo_dash::MainWindow::timerEvent(QTimerEvent *event)
{
    ui.label_time->setText(QTime::currentTime().toString("hh:mm:ss"));
}

void primo_dash::MainWindow::on_pB_s2_expoRang_2_clicked()
{
    bool ok;
    int range = QInputDialog::getInt(this, tr("Tara expo range"),
                                     tr("Enter exposure range. Range is from 10 to 1000,000 micro seconds"),100000,1,1000000,1,&ok);
    if (ok)
    {
        ui.s0e_slider->setRange(10,range);
        ui.s1e_slider->setRange(10,range);
    }
}

void primo_dash::MainWindow::on_pB_powerSensor_clicked()
{
    QString command = "roslaunch primo_base power_sensor.launch";

    // Make a confirm dialog
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Power sensor confirm", "Launch will restart power.\n "
                                                           "CONFIRM GREEN BUTTON IS BEING PRESSED BEFORE HITTING OK!\n" ,
                                    QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        QProcess process;
        QString fullCmd = "tmux new-session -d -s \"power\" " + command;
        process.startDetached(fullCmd);
    }
}

void primo_dash::MainWindow::on_pb_noUseSimTime_clicked()
{
        QProcess process;
        process.start("rosparam set /use_sim_time false",QIODevice::ReadOnly);
}

void primo_dash::MainWindow::on_pb_useSimTime_clicked()
{
        QProcess process;
        process.start("rosparam set /use_sim_time true",QIODevice::ReadOnly);
}

void primo_dash::MainWindow::on_pB_kill_map_clicked()
{
    QProcess process;
    QString commandStr = "tmux send -t map:0.0 C-c ENTER";
    process.start(commandStr,QIODevice::ReadOnly);
    process.waitForFinished();
}
