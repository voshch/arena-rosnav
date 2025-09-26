#include "task_generator_gui/task_generator_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
namespace task_generator_gui
{
    TaskGeneratorPanel::TaskGeneratorPanel(QWidget *parent) : Panel(parent)
    {
        root_layout = new QVBoxLayout(this);
    }

    TaskGeneratorPanel::~TaskGeneratorPanel() = default;

    void TaskGeneratorPanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
        node = node_ptr->get_raw_node();

        // Create a new node for the service clients
        service_node = std::make_shared<rclcpp::Node>("tm_service_node");
        // Set the log level to WARN
        node->get_logger().set_level(rclcpp::Logger::Level::Warn);
        service_node->get_logger().set_level(rclcpp::Logger::Level::Warn);
    }

    void TaskGeneratorPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        QString result;

        if (config.mapGetString("Target", &result))
        {
            task_generator_node = result.toStdString();
        }
        else
        {
            task_generator_node = "/task_generator_node";
        }

        get_environments_client = service_node->create_client<task_generator_msgs::srv::GetEnvironments>(task_generator_node + "/get_environments");

        get_parametrizeds_client = service_node->create_client<task_generator_msgs::srv::GetParametrizeds>(task_generator_node + "/get_parametrizeds");

        get_obstacles_client = service_node->create_client<task_generator_msgs::srv::GetObstacles>(task_generator_node + "/get_obstacles");

        get_scenarios_client = service_node->create_client<task_generator_msgs::srv::GetScenarios>(task_generator_node + "/get_scenarios");

        set_param_client = service_node->create_client<rcl_interfaces::srv::SetParameters>(task_generator_node + "/set_parameters");

        get_worlds_client = service_node->create_client<task_generator_msgs::srv::GetWorlds>(task_generator_node + "/get_worlds");

        get_robots_client = service_node->create_client<task_generator_msgs::srv::GetRobots>(task_generator_node + "/get_robots");

        parameters_client = std::make_shared<rclcpp::SyncParametersClient>(service_node, task_generator_node);

        generate_world_client = service_node->create_client<std_srvs::srv::Trigger>("/world_generator/generate_world");
        reset_task_client = service_node->create_client<std_srvs::srv::Empty>(task_generator_node + "/reset_task");

        getRobots();
        getWorlds();
        getTMObstaclesParams();
        getTMRobotsParams();
        getCurrentTaskGeneratorNodeParams(true);
        setupUi();
        onWorldChanged(QString::fromStdString(selected_world));
    }

    void TaskGeneratorPanel::setupUi()
    {
        // Setup Combobox for choosing Robot model
        auto robots_combobox_values = QStringList();
        for (const auto &robot : robot_models)
        {
            robots_combobox_values << QString::fromStdString(robot);
        }
        robot_combobox = setupComboBoxWithLabel(this->root_layout, robots_combobox_values, QString("Robot"));
        robot_combobox->setCurrentText(QString::fromStdString(selected_robot_model));
        connect(robot_combobox, &QComboBox::currentTextChanged, this, &TaskGeneratorPanel::onRobotChanged);
        spawn_robot_button = new QPushButton("Spawn Robot");
        connect(spawn_robot_button, &QPushButton::clicked, this, &TaskGeneratorPanel::spawnRobotButtonActivated);
        robot_combobox->parentWidget()->layout()->addWidget(spawn_robot_button);

        // Button to generate world
        generate_world_button = new QPushButton("Generate World");
        connect(generate_world_button, &QPushButton::clicked, this, &TaskGeneratorPanel::generateWorldButtonActivated);
        this->root_layout->addWidget(generate_world_button);

        // Setup Combobox for choosing World
        auto worlds_combobox_values = QStringList();
        for (const auto &world : worlds)
        {
            worlds_combobox_values << QString::fromStdString(world);
        }
        world_combobox = setupComboBoxWithLabel(this->root_layout, worlds_combobox_values, QString("World"));
        world_combobox->setCurrentText(QString::fromStdString(selected_world));
        connect(world_combobox, &QComboBox::currentTextChanged, this, &TaskGeneratorPanel::onWorldChanged);

        setupTabs(this->root_layout);

        reset_scenario_button = new QPushButton("Reset Task");
        connect(reset_scenario_button, &QPushButton::clicked, this, &TaskGeneratorPanel::resetScenarioButtonActivated);
        root_layout->addWidget(reset_scenario_button);
    }

    QComboBox *TaskGeneratorPanel::setupComboBoxWithLabel(QLayout *parent, const QStringList &combobox_values, const QString &label)
    {
        auto placeholder_widget = new QWidget();
        auto placeholder_layout = new QHBoxLayout();
        auto label_label = new QLabel(label);
        auto combobox = new QComboBox();

        combobox->addItems(combobox_values);
        placeholder_layout->addWidget(label_label);
        placeholder_layout->addWidget(combobox);
        placeholder_widget->setLayout(placeholder_layout);

        parent->addWidget(placeholder_widget);

        return combobox;
    }

    QTabWidget *TaskGeneratorPanel::setupTabs(QLayout *parent)
    {
        auto tabs = new QTabWidget();
        auto obstacles_tab_widget = new QWidget();
        auto robot_tab_widget = new QWidget();

        tabs->addTab(obstacles_tab_widget, "Obstacles");
        tabs->addTab(robot_tab_widget, "Robots");

        auto obstacles_tab_layout = new QVBoxLayout();
        auto robot_tab_layout = new QVBoxLayout();

        obstacles_task_mode_combobox = setupComboBoxWithLabel(
            obstacles_tab_layout,
            QStringList({"Environment", "Parametrized", "Random", "Scenario", "Prompt"}),
            QString("Obstacles Task Mode"));
        connect(
            obstacles_task_mode_combobox,
            &QComboBox::currentTextChanged,
            this,
            &TaskGeneratorPanel::onObstaclesTaskModeChanged);

        robot_task_mode_combobox = setupComboBoxWithLabel(
            robot_tab_layout,
            QStringList({"Explore", "Guided", "Random", "Scenario"}),
            QString("Robots Task Mode"));
        connect(robot_task_mode_combobox,
                &QComboBox::currentTextChanged,
                this,
                &TaskGeneratorPanel::onRobotsTaskModeChanged);

        obstacles_tree = setupTree(obstacles_tab_layout);
        obstacles_tree->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        obstacles_tree->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        robots_tree = setupTree(robot_tab_layout);
        robots_tree->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        robots_tree->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

        obstacles_tab_widget->setLayout(obstacles_tab_layout);
        robot_tab_widget->setLayout(robot_tab_layout);

        setupObstaclesTreeItem();
        setupRobotsTreeItem();

        parent->addWidget(tabs);

        updateTabs();
        return tabs;
    }

    void TaskGeneratorPanel::updateTabs()
    {
        obstacles_task_mode_combobox->setCurrentText(obstacles_task_mode);
        robot_task_mode_combobox->setCurrentText(robots_task_mode);
        world_combobox->setCurrentText(QString::fromStdString(selected_world));
    }

    QTreeWidget *TaskGeneratorPanel::setupTree(QLayout *parent)
    {
        auto tree = new QTreeWidget();
        tree->setColumnCount(2);
        tree->setHeaderLabels({"Parameter", "Value"});
        tree->header()->resizeSection(0, int(0.5 * tree->width()));
        tree->header()->resizeSection(1, int(0.5 * tree->width()));
        tree->setColumnWidth(0, int(0.5 * tree->width()));
        tree->setColumnWidth(1, int(0.5 * tree->width()));

        parent->addWidget(tree);

        return tree;
    }

    void TaskGeneratorPanel::onRobotChanged(const QString &text)
    {
        Q_UNUSED(text);
    }

    void TaskGeneratorPanel::onWorldChanged(const QString &text)
    {
        selected_world = text.toStdString();

        getScenarios(selected_world);
        setupObstaclesTreeItem();
        setupRobotsTreeItem();
    }

    void TaskGeneratorPanel::setupObstaclesTreeItem()
    {
        // Clear the widget tree
        obstacles_tree->clear();
        if (obstacles_task_mode == "Environment")
        {
            auto param_config_file_combobox = new QComboBox();
            param_config_file_combobox->addItems(environment_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_environment_config_file));
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_environment_config_file = text.toStdString(); });
        }

        else if (obstacles_task_mode == "Parametrized")
        {
            auto param_config_file_combobox = new QComboBox();

            param_config_file_combobox->addItems(parametrized_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_parametrized_config_file));
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_parametrized_config_file = text.toStdString(); });
        }

        else if (obstacles_task_mode == "Random")
        {
            // Set up the spinbox for n_static_obstacles
            auto n_static_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            n_static_obstacles_widgetitem->setText(0, "Number of Static Obstacles");

            RCLCPP_WARN(service_node->get_logger(), "setting up n_static_obstacles_range");
            RCLCPP_WARN(service_node->get_logger(), "size %d", int(n_static_obstacles_range.size()));
            RCLCPP_WARN(service_node->get_logger(), "size %d", int(static_obstacles_all_models.size()));
            RCLCPP_WARN(service_node->get_logger(), "n_static_obstacles_range: [%d, %d]", int(n_static_obstacles_range[0]), int(n_static_obstacles_range[1]));

            auto n_static_obstacles_widget = setupMinMaxSpinBox(&n_static_obstacles_range);
            obstacles_tree->setItemWidget(n_static_obstacles_widgetitem, 1, n_static_obstacles_widget);

            // Set up the spinbox for n_dynamic_obstacles
            auto n_dynamic_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            n_dynamic_obstacles_widgetitem->setText(0, "Number of Dynamic Obstacles");

            auto n_dynamic_obstacles_widget = setupMinMaxSpinBox(&n_dynamic_obstacles_range);
            obstacles_tree->setItemWidget(n_dynamic_obstacles_widgetitem, 1, n_dynamic_obstacles_widget);

            // Set up check boxes to choose static obstacles models
            auto static_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            static_obstacles_widgetitem->setText(0, "Static Obstacles Models");

            static_obstacles_models_groupbox = setupGroupCheckBox(static_obstacles_all_models, &static_obstacles_models_selected);
            obstacles_tree->setItemWidget(static_obstacles_widgetitem, 1, static_obstacles_models_groupbox);

            // Set up check boxes to choose dynamic obstacles models
            auto dynamic_obstacles_widgetitem = new QTreeWidgetItem(obstacles_tree);
            dynamic_obstacles_widgetitem->setText(0, "Dynamic Obstacles Models");

            dynamic_obstacles_models_groupbox = setupGroupCheckBox(dynamic_obstacles_all_models, &dynamic_obstacles_models_selected);
            obstacles_tree->setItemWidget(dynamic_obstacles_widgetitem, 1, dynamic_obstacles_models_groupbox);
        }

        else if (obstacles_task_mode == "Scenario")
        {
            auto param_config_file_combobox = new QComboBox();
            param_config_file_combobox->addItems(scenario_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_scenario_config_file));
            auto item = new QTreeWidgetItem(obstacles_tree);
            item->setText(0, "Configuration File");
            obstacles_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_scenario_config_file = text.toStdString(); });
        }

        else if (obstacles_task_mode == "Prompt")
        {
            auto use_behavior_tree_checkbox = new QCheckBox();
            use_behavior_tree_checkbox->setChecked(use_behavior_tree);
            auto use_behavior_tree_widgetitem = new QTreeWidgetItem(obstacles_tree);
            use_behavior_tree_widgetitem->setText(0, "Use Behavior Tree");
            obstacles_tree->setItemWidget(use_behavior_tree_widgetitem, 1, use_behavior_tree_checkbox);
            connect(use_behavior_tree_checkbox, &QCheckBox::stateChanged, this, [this](const bool &value)
                    { use_behavior_tree = value; });

            auto top_p_spin_box = new QDoubleSpinBox();
            top_p_spin_box->setMinimum(0.0);
            top_p_spin_box->setMaximum(1.0);
            top_p_spin_box->setSingleStep(0.1);
            top_p_spin_box->setValue(top_p);
            auto top_p_widgetitem = new QTreeWidgetItem(obstacles_tree);
            top_p_widgetitem->setText(0, "Nucleus sampling threshold (top_p)");
            obstacles_tree->setItemWidget(top_p_widgetitem, 1, top_p_spin_box);
            connect(top_p_spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](const double &value)
                    { top_p = value; });

            auto prompt_text_edit = new QTextEdit();
            prompt_text_edit->setPlaceholderText("Type your prompt here");
            prompt_text_edit->setMinimumHeight(50);
            prompt_text_edit->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
            prompt_text_edit->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
            // prompt_text_edit->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
            prompt_text_edit->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            prompt_text_edit->setLineWrapColumnOrWidth(1);
            prompt_text_edit->setLineWrapMode(QTextEdit::LineWrapMode::WidgetWidth);
            prompt_text_edit->setText(QString::fromStdString(typed_prompt));
            connect(prompt_text_edit, &QTextEdit::textChanged, this, [this, prompt_text_edit]()
                    { typed_prompt = prompt_text_edit->toPlainText().toStdString(); });

            auto prompt_widgetitem = new QTreeWidgetItem(obstacles_tree);
            prompt_widgetitem->setText(0, "Prompt");
            obstacles_tree->setItemWidget(prompt_widgetitem, 1, prompt_text_edit);
        }
    }

    void TaskGeneratorPanel::setupRobotsTreeItem()
    {
        robots_tree->clear();
        if (robots_task_mode == "Explore")
        {
        }

        else if (robots_task_mode == "Guided")
        {
        }

        else if (robots_task_mode == "Random")
        {
        }

        else if (robots_task_mode == "Scenario")
        {
            auto param_config_file_combobox = new QComboBox();
            param_config_file_combobox->addItems(scenario_config_files_qstringlist);
            param_config_file_combobox->setCurrentText(QString::fromStdString(selected_scenario_config_file));
            auto item = new QTreeWidgetItem(robots_tree);
            item->setText(0, "Configuration File");
            robots_tree->setItemWidget(item, 1, param_config_file_combobox);
            connect(param_config_file_combobox, &QComboBox::currentTextChanged, this, [this](const QString &text)
                    { selected_scenario_config_file = text.toStdString(); });
        }
    }

    QWidget *TaskGeneratorPanel::setupMinMaxSpinBox(std::vector<std::int64_t, std::allocator<std::int64_t>> *connected_values)
    {
        auto placeholder_widget = new QWidget();
        auto layout = new QHBoxLayout();

        layout->addWidget(new QLabel("Min"));

        auto min_spinbox = new QSpinBox();
        min_spinbox->setRange(0, std::numeric_limits<int>::max());
        min_spinbox->setValue(connected_values->at(0)); // Default value
        connect(min_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, connected_values](int value)
                { connected_values->at(0) = value; });
        layout->addWidget(min_spinbox);

        layout->addWidget(new QLabel("Max"));

        auto n_max_static_obstacles_spinbox = new QSpinBox();
        n_max_static_obstacles_spinbox->setRange(0, std::numeric_limits<int>::max());
        n_max_static_obstacles_spinbox->setValue(connected_values->at(1)); // Default value
        connect(n_max_static_obstacles_spinbox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, connected_values](int value)
                { connected_values->at(1) = value; });
        layout->addWidget(n_max_static_obstacles_spinbox);
        layout->addStretch();
        layout->setAlignment(Qt::AlignLeft);
        layout->setSpacing(4);

        placeholder_widget->setLayout(layout);

        return placeholder_widget;
    }

    MultiSelectComboBox *TaskGeneratorPanel::setupGroupCheckBox(std::vector<std::string> check_box_texts, std::vector<int> *connected_hash_map)
    {
        auto group_check_box = new MultiSelectComboBox();

        for (int i = 0; i < int(check_box_texts.size()); i++)
        {
            group_check_box->addItem(QString::fromStdString(check_box_texts[i]), connected_hash_map->at(i));
        }

        group_check_box->stateChanged(1);

        return group_check_box;
    }

    void TaskGeneratorPanel::onObstaclesTaskModeChanged(const QString &text)
    {
        obstacles_task_mode = text;
        getCurrentTaskGeneratorNodeParams();
        setupObstaclesTreeItem();
    }

    void TaskGeneratorPanel::onRobotsTaskModeChanged(const QString &text)
    {
        robots_task_mode = text;
        getCurrentTaskGeneratorNodeParams();
        setupRobotsTreeItem();
    }

    void TaskGeneratorPanel::generateWorldButtonActivated()
    {
        if (generateWorld())
        {
            selected_world = ".generated";
            setParams();
        }
    }
    void TaskGeneratorPanel::resetScenarioButtonActivated()
    {
        setParams();
    }
    void TaskGeneratorPanel::spawnRobotButtonActivated()
    {
        setRobot();
    }

} // namespace task_generator_gui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(task_generator_gui::TaskGeneratorPanel, rviz_common::Panel)