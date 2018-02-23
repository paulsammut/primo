/**
 * @file /include/primo_dash/main_window.hpp
 *
 * @brief Qt based gui for primo_dash.
 *
 * @date November 2010
 **/
#ifndef primo_dash_MAIN_WINDOW_H
#define primo_dash_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace primo_dash {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private Q_SLOTS:
    void on_pB_s2_disableXinput_clicked();

    void on_pB_s2_viewLeft_clicked();

    void on_pB_s2_viewRight_clicked();

    void on_pB_robonet_vpn_primo_clicked();

    void on_pB_robonet_shop_primo_clicked();

    void on_pB_robonet_field_primo_clicked();

    void on_pB_robonet_vpn_laptop_clicked();

    void on_pB_robonet_shop_laptop_clicked();

    void on_pB_robonet_field_laptop_clicked();

    void on_pB_roscore_clicked();

    void on_pB_rviz_clicked();

    void on_pB_clearCostmap_clicked();

    void on_pB_cpMap_clicked();

    void on_pB_qtcam_clicked();

    void on_pB_killQtcam_clicked();

    void on_s2le_slider_sliderMoved(int position);

    void on_s2lb_spinBox_editingFinished();

    void on_s2le_spinBox_editingFinished();

    void on_s2lb_slider_sliderMoved(int position);

    void on_s2re_slider_sliderMoved(int position);

    void on_s2re_spinBox_editingFinished();

    void on_s2rb_slider_sliderMoved(int position);

    void on_s2rb_spinBox_editingFinished();

    void on_s2_trig_slider_sliderMoved(int position);

    void on_s2_trig_spinBox_editingFinished();

    void on_s0e_slider_sliderMoved(int position);

    void on_s0e_spinBox_editingFinished();

    void on_s0b_spinBox_editingFinished();

    void on_s0b_slider_sliderMoved(int position);

    void on_s1e_slider_sliderMoved(int position);

    void on_s1e_spinBox_editingFinished();

    void on_s1b_slider_sliderMoved(int position);

    void on_s1b_spinBox_editingFinished();

    void on_pb_launch_base_alpha_clicked();

    void on_pushButton_4_clicked();

    void on_pB_stereo_suite_clicked();

    void on_pB_killCostmap_clicked();

    void on_pB_qtcam_2_clicked();

    void on_pB_qtcam_3_clicked();

    void on_pB_killRos_clicked();

    void on_pB_color0_clicked();

    void on_pB_view_s1_right_clicked();

    void on_pB_view_s1_left_clicked();

    void on_pB_view_s0_right_clicked();

    void on_pB_view_s0_left_clicked();

    void on_pb_kill_rqt_view_clicked();

    void on_pB_kill_rqt_view2_clicked();

    void on_pB_localization_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    /**
     * @brief Helper function that laungers robonet with the given arguments.
     * Robonet must be placed in the sudoers files for it to work.
     *
     * @param robonetArgs The robonet args that must be in the form "-n NETWORK
     * -r ROSMASTER_HOST. See robonet help for more info.
     * 
     */
    void launchRobonet(QString robonetArgs);
};

}  // namespace primo_dash

#endif // primo_dash_MAIN_WINDOW_H
