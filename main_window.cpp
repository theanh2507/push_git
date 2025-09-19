#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <rviz_rendering/render_window.hpp>
#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>

#include <QProcess>
#include <QDir>

#include "rclcpp/rclcpp.hpp"

#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>


MainWindow::MainWindow(QApplication * app, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _app(app)
{
    ui->setupUi(this);              // this: Mainwindow da khoi tao

    initial();
    setupRobotModelDisplay();
    setmap();
    set_tool();

    auto raw_node = _rvizRosNodeTmp->get_raw_node();                    // tra ve kieu std::shared_ptr<rclcpp::Node>

    // gui ten cua table sang waypoint
    table_publisher = raw_node->create_publisher<std_msgs::msg::String>("selected_table", 10);

    confirm = raw_node->create_subscription<std_msgs::msg::String>("status_robot", 10, std::bind(&MainWindow::confirm_callback, this, std::placeholders::_1));

    finish_publisher = raw_node->create_publisher<std_msgs::msg::String>("finish_node", 10);

    vel_publisher = raw_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pub_move_multiple_point = raw_node->create_publisher<std_msgs::msg::Bool>("/start_navigation",10);

    pause_pub = raw_node->create_publisher<std_msgs::msg::Bool>("/pause_cmd",10);

    connect(ui->comboBox, &QComboBox::currentTextChanged, this, &MainWindow::setmap);
    
    connect(ui->pushButton_14, &QPushButton::clicked, this, [=](){
        setView("2D");
    });
    
    connect(ui->pushButton_15, &QPushButton::clicked, this, [=](){
        setView("3D");
    });

    connect(ui->pushButton_20, &QPushButton::clicked, this, [=]() {
        _manager->getToolManager()->setCurrentTool(initial_pose_tool);
    });

    connect(ui->pushButton_6, &QPushButton::clicked, this, [=]() {
        _manager->getToolManager()->setCurrentTool(nav_goal_tool);
    });

    connect(ui->pushButton_27, &QPushButton::clicked, this, [=]() {
        _manager->getToolManager()->setCurrentTool(publish_point_tool);
    });


    // control robot manual
    connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::Control_Robot_Manual);

    // nhap ten map moi cho save
    ui->pushButton_25->setDisabled(ui->lineEdit->text().trimmed().isEmpty());
    connect(ui->lineEdit, &QLineEdit::textChanged, this, [=](const QString &text) {
    ui->pushButton_25->setDisabled(text.trimmed().isEmpty());
    });


    connect(ui->pushButton_19, &QPushButton::clicked, this, &MainWindow::select_table);
    connect(ui->pushButton_22, &QPushButton::clicked, this, &MainWindow::remove_table);
    connect(ui->pushButton_23, &QPushButton::clicked, this, &MainWindow::remove_all_table);
    connect(ui->pushButton_21, &QPushButton::clicked, this, &MainWindow::move_table);
    connect(ui->pushButton_18, &QPushButton::clicked, this, &MainWindow::move_finish);

    // connect button to slam
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::start_slam);
    connect(ui->pushButton_24,&QPushButton::clicked, this, &MainWindow::quit_slam);
    connect(ui->pushButton_25,&QPushButton::clicked, this, &MainWindow::save_map);


    // button multiple point
    connect(ui->pushButton_28,&QPushButton::clicked, this, &MainWindow::start_multiple_point);

    // pause, resume cmd_vel
    connect(ui->pushButton_16,&QPushButton::clicked, this, &MainWindow::pause_robot);
    connect(ui->pushButton_17,&QPushButton::clicked, this, &MainWindow::resume_robot);
}

MainWindow::~MainWindow()
{
    quit_slam();
    delete ui;
    kill(0, SIGINT);                            // dung toan bo tien trinh
}


void MainWindow::initial()
{
    _rvizRosNodeTmp = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("gui_node");
    _rvizRosNode = _rvizRosNodeTmp;

    QApplication::processEvents();                                          // tranh treo UI trong luc khoi tao rviz
    _render_panel = std::make_shared<rviz_common::RenderPanel>();           // khoi tao rendel panel hien thi rviz
    QApplication::processEvents();
    _render_panel->getRenderWindow()->initialize();

    rviz_common::WindowManagerInterface * wm = nullptr;
    auto clock = _rvizRosNode.lock()->get_raw_node()->get_clock();
    _manager = std::make_shared<rviz_common::VisualizationManager>(_render_panel.get(), _rvizRosNode, wm, clock);       // _render_panel.get(): tra ve 1 con tro      
    _render_panel->initialize(_manager.get());
    QApplication::processEvents();

    _manager->setFixedFrame("/map");
    _manager->initialize();
    _manager->startUpdate();

    ui->scrollArea->setWidget(_render_panel.get());

    // 添加网格显示
    _grid = _manager->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
    if (_grid == NULL) {
        throw std::runtime_error("Error creating grid display");
    }

    // 配置网格样式
    _grid->subProp("Line Style")->setValue("Billboards");
    _grid->subProp("Line Style")->subProp("Line Width")->setValue(0.02f);
    _grid->subProp("Color")->setValue(QColor(Qt::white));
    _grid->subProp("Cell Size")->setValue(1.0f);

    // 添加点云显示
    _pointcloud = _manager->createDisplay("rviz_default_plugins/LaserScan", "scan", true);
    if (_pointcloud == NULL) {
        throw std::runtime_error("Error creating pointcloud display");
    }

    // 配置点云样式
    _pointcloud->subProp("Topic")->setValue("/scan_filter");               // gazebo topic lidar: /laser_controller/out    real topic lidar: /scan_filter
    _pointcloud->subProp("Style")->setValue("Points");
    _pointcloud->subProp("Size (Pixels)")->setValue(2);
    _pointcloud->subProp("Color Transformer")->setValue("Intensity");
    _pointcloud->subProp("Invert Rainbow")->setValue("true");
    _pointcloud->subProp("Decay Time")->setValue("0.1");


    // Path
    path = _manager->createDisplay("rviz_default_plugins/Path", "plan", true);
    if (path == NULL) {
        throw std::runtime_error("Error creating path display");
    }
    path->subProp("Topic")->setValue("/plan");
    path->subProp("Line Style")->setValue("Billboards");
    path->subProp("Line Width")->setValue(0.1);
    path->subProp("Color")->setValue("0 255 0");

    // Multiple point display
    multiple_point = _manager->createDisplay("rviz_default_plugins/Marker", "/multiple point", true);
    multiple_point->subProp("Topic")->setValue("/visualization_marker");


    // ##########################################
    _render_panel->setMouseTracking(true);
    _render_panel->setFocusPolicy(Qt::StrongFocus);

    // Set the view controller to Orbit to allow for mouse interactions
    _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");

    // Retrieve the active view controller to set properties and confirm it's set up correctly
    auto orbit_view_controller = _manager->getViewManager()->getCurrent();
    if (!orbit_view_controller) {
        qDebug() << "Orbit view controller could not be set.";
        return;
    }

    qDebug() << "Orbit view controller initialized successfully.";

    // Set default distance and focal point for the camera
    orbit_view_controller->subProp("Distance")->setValue(10.0);
    orbit_view_controller->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));

    // Set initial orientation of the camera
    orbit_view_controller->subProp("Pitch")->setValue(1.5708);  // Example angle in radians
    orbit_view_controller->subProp("Yaw")->setValue(3.14);      // Example angle in radians

    // Set Interact tool as the active tool to enable mouse interactions
    auto tool_manager = _manager->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));


    ///////////////////////////// robot process /////////////////////////////
    // if(robot_process)
    // {
    //     robot_process->kill();
    //     robot_process->waitForFinished(3000);
    //     delete robot_process;
    //     robot_process = nullptr;
    // }

    robot_process = new QProcess(this);

    QStringList robot_args;

    robot_args << "launch"
            << "agv_test_pkg"
            << "launch_sim.launch.py";
    
    robot_process->setProgram("ros2");
    robot_process->setArguments(robot_args);
    robot_process->start();

    ///////////////////////////// multiple point process /////////////////////////////
    multiple_point_process = new QProcess(this);
    QStringList multiple_point_args;

    multiple_point_args << "run"
                        << "agv_test_pkg"                                                      // gazebo: articubot_one, real_robot: agv_test_pkg
                        << "multiple_point.py";

    multiple_point_process->setProgram("ros2");
    multiple_point_process->setArguments(multiple_point_args);
    multiple_point_process->start();

    ///////////////////////////// send position table process /////////////////////////////
    send_position_table_process = new QProcess(this);

    QStringList args;
    args << "run"
        << "agv_test_pkg"
        << "waypoint.py"
        << "--ros-args"
        << "--params-file"
        << "/home/theanh/agv_test_ws/src/agv_test_pkg/config/waypoint_follow.yaml";

    send_position_table_process->setProgram("ros2");
    send_position_table_process->setArguments(args);
    send_position_table_process->start();
}


void MainWindow::setupRobotModelDisplay()
{
    robot_model_display = _manager->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    
    if(robot_model_display)
    {
        robot_model_display -> subProp("Description Topic")->setValue("/robot_description");
        qDebug() << "RobotModel display configured for /robot_description topic.";
    }

    else
    {
        qDebug() << "Failed to create RobotModel display.";
    }
}

void MainWindow::setmap()
{
    auto name_map = ui->comboBox->currentText();

    QString map_yaml;
    if (name_map == "map1")
        map_yaml = "/home/theanh/maps/test_map.yaml";
    else if (name_map == "map2")
        map_yaml = "/home/theanh/maps/map_antue_25_6.yaml";
    else if (name_map == "map3")
        map_yaml = "/home/theanh/maps/map_nta.yaml";
    else
        return;
    
    ////////// localization process
    if(localization_process)
    {
        localization_process->kill();
        localization_process->waitForFinished(3000);
        delete localization_process;
        localization_process = nullptr;
    }

    localization_process = new QProcess(this);

    QStringList localization_args;

    localization_args << "launch"
                      << "agv_test_pkg"                                                            // gazebo: articubot_one, real_robot: agv_test_pkg
                      << "localization_launch.py"
                      << QString("map:=%1").arg(map_yaml);

    // localization_args << "launch"
    //                   << "agv_test_pkg"
    //                   << "localization_launch.py"
    //                   << QString("map:=%1").arg(map_yaml);
    
    localization_process->setProgram("ros2");
    localization_process->setArguments(localization_args);
    localization_process->start();


    if (!localization_process->waitForStarted(3000)) {
        qDebug() << "can't startlocalization_process";
        return;
        }
    qDebug() << "localization_process start";


    ////////// navigation process
    if(navigation_process)
    {
        navigation_process->kill();
        navigation_process->waitForFinished(3000);
        delete navigation_process;
        navigation_process = nullptr;
    }

    navigation_process = new QProcess(this);

    QStringList navigation_args;

    navigation_args << "launch"
                    << "agv_test_pkg"                                                              // gazebo: articubot_one, real_robot: agv_test_pkg
                    << "navigation_launch.py";
    
    navigation_process->setProgram("ros2");
    navigation_process->setArguments(navigation_args);
    navigation_process->start();


    // Config display map
    auto map_display = _manager->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display)
    {
        map_display->subProp("Topic")->setValue("/map");
        qDebug() << "RViz show map.";
        QApplication::processEvents();
    }
    else
    {
        qDebug() << "Can't create map";
    }
}


void MainWindow::setView(const QString &view_mode)
{
    if(view_mode == "2D")
    {
        _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");
        _manager->getViewManager()->getCurrent()->subProp("Scale")->setValue(40);
    }

    else if(view_mode == "3D")
    {
        _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
    }
    else
    {
        qDebug() << "Fail to set View";
    }
}

void MainWindow::set_tool()
{
    auto tool_manager = _manager->getToolManager();

    initial_pose_tool = tool_manager->addTool("rviz_default_plugins/SetInitialPose");
    nav_goal_tool = tool_manager->addTool("rviz_default_plugins/SetGoal");
    publish_point_tool = tool_manager->addTool("rviz_default_plugins/PublishPoint");

    initial_pose_tool->setName("InitialPose");
    nav_goal_tool->setName("NavGoal");
    publish_point_tool->setName("PublishPoint");
}

void MainWindow::select_table()
{
    auto name_table = ui->comboBox_2->currentText();
    if(!table_list.contains(name_table) && !name_table.isEmpty())
    {
        table_list.append(name_table);
        ui->listWidget->addItem(name_table);
        qDebug()<<"Add table: "<<name_table;
    }
    else
    {
        qDebug()<<"Table already in list";
    }
    qDebug()<<"data table: "<<table_list;  
}


void MainWindow::move_table()
{
    if(table_list.isEmpty())
    {
        qDebug()<<"Emty list";
    }

    else
    {
        QString target = table_list.join(",");
        auto message = std_msgs::msg::String();
        message.data = target.toStdString();
        table_publisher->publish(message);
        table_list.clear();
    }
    remove_all_table();
}


void MainWindow::remove_table()
{
    QList<QListWidgetItem*> selectedItems = ui->listWidget->selectedItems();
    for (QListWidgetItem* item : selectedItems)
    {
        QString name_table = item->text();
        table_list.removeAll(name_table);

        delete ui->listWidget->takeItem(ui->listWidget->row(item));
    }
    qDebug()<<"data table: "<<table_list;   
}

void MainWindow::remove_all_table()
{
    ui->listWidget->clear();
    table_list.clear();
    qDebug()<<"data table: "<<table_list;  
}

void MainWindow::confirm_callback(const std_msgs::msg::String::SharedPtr msg)
{
    QString data = QString::fromStdString(msg->data);
    ui->label_6->setText(data);
}

void MainWindow::move_finish()
{
    auto msg = std_msgs::msg::String();
    msg.data = "go_to_next_point";
    finish_publisher->publish(msg);
}

void MainWindow::start_slam()
{
    QProcess::execute("pkill", QStringList() << "-f" << "slam_toolbox");        // tat cac tien trinh cu
    slam_process = new QProcess(this);

    QStringList slam_args;

    slam_args << "launch"   
              << "agv_test_pkg"                                                // gazebo: articubot_one, real_robot: agv_test_pkg
              << "online_async_launch.py"
              << "use_sim_time:=True";

    // slam_args << "launch"
    //         << "agv_test_pkg"
    //         << "online_async_launch.py";

    slam_process->setProgram("ros2");
    slam_process->setArguments(slam_args);
    slam_process->start();

    // slam_process->setArguments(slam_args);
    // slam_process->start("setsid", slam_args);

    // Config display map
    auto map_display = _manager->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display)
    {
        map_display->subProp("Topic")->setValue("/map");
        qDebug() << "RViz show map.";
        QApplication::processEvents();
    }
    else
    {
        qDebug() << "Can't create map.";
    }
}


void MainWindow::quit_slam()
{
    // Tạo tiến trình để lấy PID từ lệnh ps + grep + awk
    QProcess find_pid;
    // awk '{print $1}': in ra cot dau tien (process id cua slamtoolbox)
    find_pid.start("bash", QStringList() << "-c" << "ps -eo pid,command | grep slam_toolbox | grep -v grep | awk '{print $1}'");
    find_pid.waitForFinished();
    
    QString pid_str = QString(find_pid.readAllStandardOutput()).trimmed();

    if (!pid_str.isEmpty())
    {
        qDebug() << "Found PID:" << pid_str;

        // Gửi SIGINT đến PID đã tìm
        QProcess::execute("kill", QStringList() << "-SIGINT" << pid_str);

        QProcess::execute("pkill", QStringList() << "-f" << "slam_toolbox"); 
    }
    else
    {
        qDebug() << "slam_toolbox not running or PID not found.";
    }

    auto root_display_group = _manager->getRootDisplayGroup();

    int num_displays = root_display_group->numDisplays();
    for (int i = 0; i < num_displays; ++i)
    {
        auto display = root_display_group->getDisplayAt(i);
        if (display && display->getName().toStdString() == "Map Display")  // check dung name map cua create_display da tao
        {
            display->setEnabled(false);  // hoặc root_display_group->removeChild(display);
            break;
        }
    }

    // QProcess* kill_process = new QProcess;

    // QString pid = QString::number(slam_process->processId());
    // QString program = "kill";
    // QStringList arguments;
    // arguments << "-SIGINT" << pid;

    // qDebug() << program << arguments;

    // kill_process->start(program, arguments);

    // bool started = kill_process->waitForStarted();
    // qDebug() << "Process started:" << started << kill_process->errorString();
    // Q_ASSERT(started);
}

void MainWindow::save_map()
{
    save_map_process = new QProcess(this);
    QString map_name = ui->lineEdit->text().trimmed();                  // trimmed: loai bo khoang trang o dau va cuoi chuoi

    QDir().mkpath(QDir::homePath() + "/maps");
    QString full_path = QDir::homePath() + "/maps/" + map_name;

    QStringList save_map_args;
    save_map_args << "run"
                  << "nav2_map_server"
                  << "map_saver_cli"
                  << "-f"
                  << full_path;

    save_map_process->start("ros2", save_map_args);
}


void MainWindow::Control_Robot_Manual()
{
    control_enable = !control_enable;
    if(control_enable)
    {
        ui->pushButton_9->setText("Start");
        ui->pushButton_9->setStyleSheet("background-color: green; color: white;");

        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->pushButton_4->setEnabled(true);
        
        // this: doi tuong nhan tin hieu(MainWindow), [this]: ham thuc thi khi co signal
        connect(ui->pushButton, &QPushButton::pressed, this, [this]() {                       // thang                   
            currentTwist.linear.x = ui->doubleSpinBox->value(); 
            currentTwist.angular.z = 0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_4, &QPushButton::pressed, this, [this]() {                     // lui       
            currentTwist.linear.x = -(ui->doubleSpinBox->value()); 
            currentTwist.angular.z = 0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_3, &QPushButton::pressed, this, [this]() {                     // phai         
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = -(ui->doubleSpinBox_2->value()); 
            SendCommand_Vel();
        });

        connect(ui->pushButton_2, &QPushButton::pressed, this, [this]() {                     // trai     
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = ui->doubleSpinBox_2->value(); 
            SendCommand_Vel();
        });

        connect(ui->pushButton, &QPushButton::released, this, [this]() {                      // nha                 
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_2, &QPushButton::released, this, [this]() {                                    
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_3, &QPushButton::released, this, [this]() {                                
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_4, &QPushButton::released, this, [this]() {                                     
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });
    }
    else
    {
        ui->pushButton_9->setText("Stop");
        ui->pushButton_9->setStyleSheet("background-color: red; color: white;");
        
        // tat dieu khien robot
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(false);
        ui->pushButton_3->setEnabled(false);
        ui->pushButton_4->setEnabled(false);
    }
    
}


void MainWindow::SendCommand_Vel()
{
    vel_publisher->publish(currentTwist);
}

void MainWindow::start_multiple_point()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub_move_multiple_point->publish(msg);
}

void MainWindow::pause_robot()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    pause_pub->publish(msg);  // topic: /pause_cmd
}

void MainWindow::resume_robot()
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    pause_pub->publish(msg);
}


// void MainWindow::delete_map()
// {
//     pass
// }

// void MainWindow::quit_slam()
// {
//     if (slam_process && slam_process->state() != QProcess::NotRunning)
//     {
//         qint64 pid = slam_process->processId();
//         QString pidStr = QString::number(pid);

//         // Gửi tín hiệu SIGINT để tắt sạch sẽ node
//         QProcess::execute("kill", QStringList() << "-SIGINT" << pidStr);

//         // Đợi node shutdown
//         slam_process->waitForFinished(3000);

//         delete slam_process;
//         slam_process = nullptr;
//     }
// }
