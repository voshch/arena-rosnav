#include "task_generator_gui/task_generator_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include <cctype>

namespace task_generator_gui
{
    void TaskGeneratorPanel::getRobots()
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetRobots::Request>();

        auto response = sendRequest<task_generator_msgs::srv::GetRobots>(get_robots_client, request, "get_robots");

        robot_models = response->robots;
        selected_robot_model = robot_models.empty() ? "" : robot_models[0];
    }

    void TaskGeneratorPanel::getWorlds()
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetWorlds::Request>();

        auto response = sendRequest<task_generator_msgs::srv::GetWorlds>(get_worlds_client, request, "get_worlds");

        worlds = response->worlds;
        selected_world = worlds.empty() ? "" : worlds[0];
    }

    void TaskGeneratorPanel::getCurrentTaskGeneratorNodeParams(bool init)
    {
        if (!parameters_client)
        {
            RCLCPP_WARN(rclcpp::get_logger("TaskGeneratorPanel"),
                        "Parameter client is not available.");
            return;
        }

        while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while watiting for parameters_client service!. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Service is not available, waiting again...");
        }

        try
        {
            RCLCPP_INFO(service_node->get_logger(), "Getting parameters from /task_generator_node");
            if(init){
                auto tm_obstacles_param = parameters_client->get_parameter<std::string>("tm_obstacles");
                if(!tm_obstacles_param.empty()){
                    tm_obstacles_param[0] = std::toupper(static_cast<unsigned char>(tm_obstacles_param[0]));
                }
                obstacles_task_mode = QString::fromStdString(tm_obstacles_param);

                auto tm_robots_param = parameters_client->get_parameter<std::string>("tm_robots");
                if(!tm_robots_param.empty()){
                    tm_robots_param[0] = std::toupper(static_cast<unsigned char>(tm_robots_param[0]));
                }
                robots_task_mode = QString::fromStdString(tm_robots_param);
            }
            auto current_obstacles_tm = obstacles_task_mode.toStdString();
            for (char &c : current_obstacles_tm) {
                c = std::tolower(static_cast<unsigned char>(c));
            }

            auto current_robots_tm = robots_task_mode.toStdString();
            for (char &c : current_robots_tm) {
                c = std::tolower(static_cast<unsigned char>(c));
            }

            RCLCPP_WARN(service_node->get_logger(), "Current Obstacles Task Mode: %s", current_obstacles_tm.c_str());
            RCLCPP_WARN(service_node->get_logger(), "Current Robots Task Mode: %s", current_robots_tm.c_str());

            // rclcpp::SyncParametersClient::SharedPtr does not support nested parameters for has_parameter function
            if (current_obstacles_tm == "environment")
            {
                auto config_file = parameters_client->get_parameter<std::string>("task.environment.file", environment_config_files.empty() ? "" : environment_config_files[0]);
                selected_environment_config_file = config_file;
            }
            if (current_obstacles_tm == "parametrized")
            {
                auto config_file = parameters_client->get_parameter<std::string>("task.parametrized.file", parametrized_config_files.empty() ? "" : parametrized_config_files[0]);
                selected_parametrized_config_file = config_file;
            }
            if (current_obstacles_tm == "random")
            {
                auto current_static_models = parameters_client->get_parameter<std::vector<std::string>>("task.random.static.models", {});
                auto current_dynamic_models = parameters_client->get_parameter<std::vector<std::string>>("task.random.dynamic.models", {});

                for (size_t i = 0; i < static_obstacles_all_models.size(); i++)
                {
                    for (auto &model : current_static_models)
                    {
                        if (static_obstacles_all_models[i] == model)
                        {
                            static_obstacles_models_selected[i] = 1;
                            RCLCPP_INFO(service_node->get_logger(), "Static obstacles model selected: %s", static_obstacles_all_models[i].c_str());
                        }
                    }
                }

                for (size_t i = 0; i < dynamic_obstacles_all_models.size(); i++)
                {
                    for (auto &model : current_dynamic_models)
                    {
                        if (dynamic_obstacles_all_models[i] == model)
                        {
                            dynamic_obstacles_models_selected[i] = 1;
                            RCLCPP_INFO(service_node->get_logger(), "Dynamic obstacles model selected: %s", dynamic_obstacles_all_models[i].c_str());
                        }
                    }
                }

                RCLCPP_INFO(service_node->get_logger(), "Static obstacles models selected: ");

                n_static_obstacles_range = parameters_client->get_parameter<std::vector<int64_t>>("task.random.static.n", {0, 0});
                n_dynamic_obstacles_range = parameters_client->get_parameter<std::vector<int64_t>>("task.random.dynamic.n", {0, 0});
                RCLCPP_WARN(service_node->get_logger(), "got ranges");
                RCLCPP_WARN(service_node->get_logger(), "n_static_obstacles_range: [%d, %d]", int(n_static_obstacles_range[0]), int(n_static_obstacles_range[1]));
            }
            if (current_obstacles_tm == "scenario")
            {
                auto config_file = parameters_client->get_parameter<std::string>("task.scenario.file", scenario_config_files.empty() ? "" : scenario_config_files[0]);

                selected_scenario_config_file = config_file;
            }
            if (current_obstacles_tm == "prompt")
            {
                auto prompt = parameters_client->get_parameter<std::string>("task.prompt.user_prompt", "One pedestrian walking along the walls.");

                typed_prompt = prompt;

                auto use_bt = parameters_client->get_parameter<bool>("task.prompt.behavior_tree", false);

                use_behavior_tree = use_bt;

                auto p = parameters_client->get_parameter<double>("task.prompt.top_p", 0.3);

                top_p = p;
            }


            RCLCPP_INFO(service_node->get_logger(), "Current Robot Task Mode: %s", current_robots_tm.c_str());

            if (parameters_client->has_parameter("robot"))
            {
                auto current_robot_model = parameters_client->get_parameter<std::string>("robot");
                selected_robot_model = current_robot_model;
            }

            if (parameters_client->has_parameter("world"))
            {
                auto current_world = parameters_client->get_parameter<std::string>("world");
                selected_world = current_world;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get parameters: %s", e.what());
        }
    }

    void TaskGeneratorPanel::getTMObstaclesParams()
    {
        try
        {
            // Get configs for Environment Obstacles Task Mode
            auto environment_request = std::make_shared<task_generator_msgs::srv::GetEnvironments::Request>();

            auto environment_response = sendRequest<task_generator_msgs::srv::GetEnvironments>(get_environments_client, environment_request, "get_environments");
            environment_config_files = environment_response->environments;

            environment_config_files_qstringlist = QStringList();
            for (const auto &environment : environment_config_files)
            {
                environment_config_files_qstringlist << QString::fromStdString(environment);
            }

            // Get configs for Parametrized Obstacles Task Mode
            auto parametrized_request = std::make_shared<task_generator_msgs::srv::GetParametrizeds::Request>();

            auto parametrized_response = sendRequest<task_generator_msgs::srv::GetParametrizeds>(get_parametrizeds_client, parametrized_request, "get_parametrizeds");

            parametrized_config_files = parametrized_response->parametrizeds;

            parametrized_config_files_qstringlist = QStringList();
            for (const auto &parametrized : parametrized_config_files)
            {
                parametrized_config_files_qstringlist << QString::fromStdString(parametrized);
            }

            // Get configs for Random Obstacles Task Mode
            auto obstacles_request = std::make_shared<task_generator_msgs::srv::GetObstacles::Request>();

            auto obstacles_response = sendRequest<task_generator_msgs::srv::GetObstacles>(get_obstacles_client, obstacles_request, "get_models");

            static_obstacles_all_models = obstacles_response->models_static_obstacles;
            static_obstacles_models_selected = std::vector<int>(static_obstacles_all_models.size(), 0);
            dynamic_obstacles_all_models = obstacles_response->models_dynamic_obstacles;
            dynamic_obstacles_models_selected = std::vector<int>(dynamic_obstacles_all_models.size(), 0);

            // Get configs for Scenario Obstacles Task Mode
            getScenarios(selected_world);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void TaskGeneratorPanel::getScenarios(const std::string &world_name)
    {
        auto request = std::make_shared<task_generator_msgs::srv::GetScenarios::Request>();
        request->world = world_name;

        auto response = sendRequest<task_generator_msgs::srv::GetScenarios>(get_scenarios_client, request, "get_scenarios");
        scenario_config_files = response->scenarios;

        scenario_config_files_qstringlist = QStringList();
        for (const auto &scenario : scenario_config_files)
        {
            scenario_config_files_qstringlist << QString::fromStdString(scenario);
        }
    }

    void TaskGeneratorPanel::getTMRobotsParams()
    {
        try
        {
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void TaskGeneratorPanel::setTMObstaclesParamsRequest()
    {
        RCLCPP_WARN(service_node->get_logger(), "Setting params for Obstacles Task Mode: %s", obstacles_task_mode.toStdString().c_str());

        if (obstacles_task_mode == "Environment")
        {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "environment";
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            if (!hasNestedParameter("task.environment.file"))
            { // if the param not exist, this will reset the task so the param will be available and its value can be set
                auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
                sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");
            }

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.environment.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_environment_config_file;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
        }
        else if (obstacles_task_mode == "Parametrized")
        {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "parametrized";
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            if (!hasNestedParameter("task.parametrized.file")) // if the param not exist, this will reset the task so the param will be available and its value can be set
            {
                auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
                sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");
            }

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.parametrized.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_parametrized_config_file;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
        }
        else if (obstacles_task_mode == "Random")
        {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "random";
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            if (!hasNestedParameter("task.random.static.models"))
            { // if the param not exist, this will reset the task so the param will be available and its value can be set
                auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
                sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");
            }

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.static.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_static_obstacles_models = convert(static_obstacles_models_groupbox->currentText());
            parameter.value.string_array_value = selected_static_obstacles_models;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.dynamic.models";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            std::vector<std::string> selected_dynamic_obstacles_models = convert(dynamic_obstacles_models_groupbox->currentText());
            parameter.value.string_array_value = selected_dynamic_obstacles_models;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.static.n";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            parameter.value.integer_array_value = n_static_obstacles_range;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.random.dynamic.n";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
            parameter.value.integer_array_value = n_dynamic_obstacles_range;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
        }
        else if (obstacles_task_mode == "Scenario")
        {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "scenario";
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            if (!hasNestedParameter("task.scenario.file"))
            { // if the param not exist, this will reset the task so the param will be available and its value can be set
                auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
                sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");
            }

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.scenario.file";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = selected_scenario_config_file;
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
        }
        else if (obstacles_task_mode == "Prompt")
        {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_obstacles";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "prompt";
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            if (!hasNestedParameter("task.prompt.user_prompt"))
            { // if the param not exist, this will reset the task so the param will be available and its value can be set
                auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
                sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");
            }

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.prompt.user_prompt";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = typed_prompt;
            RCLCPP_WARN(service_node->get_logger(), "typed_prompt: %s", parameter.value.string_value.c_str());
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.prompt.behavior_tree";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            parameter.value.bool_value = use_behavior_tree;
            RCLCPP_WARN(service_node->get_logger(), "use_behavior_tree: %d", parameter.value.bool_value);
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

            request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            parameter = rcl_interfaces::msg::Parameter();
            parameter.name = "task.prompt.top_p";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            parameter.value.double_value = top_p;
            RCLCPP_WARN(service_node->get_logger(), "top_p: %lf", parameter.value.double_value);
            request->parameters.push_back(parameter);
            sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
        }
    }

    void TaskGeneratorPanel::setTMRobotsParamsRequest()
    {
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        if (robots_task_mode == "Explore")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "explore";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Guided")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "guided";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Random")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "random";
            request->parameters.push_back(parameter);
        }
        else if (robots_task_mode == "Scenario")
        {
            rcl_interfaces::msg::Parameter parameter;
            parameter.name = "tm_robots";
            parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter.value.string_value = "scenario";
            request->parameters.push_back(parameter);
        }
        sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");
    }

    bool TaskGeneratorPanel::generateWorld()
    {
        auto generate_world_future = generate_world_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (rclcpp::spin_until_future_complete(service_node, generate_world_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Successfully generated world");
            return true;
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to generate world");
            return false;
        }
    }

    void TaskGeneratorPanel::getParams()
    {
        getRobots();
        getWorlds();
        getCurrentTaskGeneratorNodeParams(false);
        updateTabs();
        getTMObstaclesParams();
        getTMRobotsParams();
    }

    void TaskGeneratorPanel::setParams()
    {
        while (!set_param_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Waiting for set_parameters service...");
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::Parameter world_param;
        world_param.name = "world";
        world_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        world_param.value.string_value = selected_world;
        request->parameters.push_back(world_param);
        sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

        request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter robot_model_param;
        robot_model_param.name = "robot";
        robot_model_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        robot_model_param.value.string_value = selected_robot_model;
        request->parameters.push_back(robot_model_param);
        sendRequest<rcl_interfaces::srv::SetParameters>(set_param_client, request, "set_param");

        // auto lowered_obstacles_taskmode = obstacles_task_mode.toStdString();
        // lowered_obstacles_taskmode[0] = std::tolower(static_cast<unsigned char>(lowered_obstacles_taskmode[0]));

        setTMObstaclesParamsRequest();
        setTMRobotsParamsRequest();

        auto reset_task_request = std::make_shared<std_srvs::srv::Empty::Request>();
        sendRequest<std_srvs::srv::Empty>(reset_task_client, reset_task_request, "reset_task");

        getParams();
    }

    void TaskGeneratorPanel::setRobot()
    {
        while (!set_param_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(service_node->get_logger(), "Waiting for set_parameters service...");
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::Parameter robot_param;
        robot_param.name = "robot";
        robot_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        checkRobotModel();
        robot_param.value.string_value = selected_robot_model;
        request->parameters.push_back(robot_param);

        auto future = set_param_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(service_node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(), "Successfully set robot models");
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(), "Failed to set robot models");
        }
    }

    std::vector<std::string> TaskGeneratorPanel::convert(const QStringList &qList)
    {
        std::vector<std::string> result;
        result.reserve(qList.size()); // optional, for efficiency
        for (const QString &item : qList)
        {
            result.push_back(item.toStdString());
        }
        return result;
    }

    void TaskGeneratorPanel::checkRobotModel()
    // Check if the choosen robot in the combobox is already set in the /task_generator_node param
    // Set up a string value for the /task_generator_node/robot parameter
    {
        std::stringstream ss(selected_robot_model);
        std::string temp;
        auto choosen_robot = robot_combobox->currentText().toStdString();

        RCLCPP_INFO(service_node->get_logger(), "Selected robot model: %s", choosen_robot.c_str());
        // // forbid duplicate robots
        // while (getline(ss, temp, del))
        // {
        //     if (temp == choosen_robot)
        //         return;
        // }
        selected_robot_model.append(",").append(choosen_robot);
    }

    bool TaskGeneratorPanel::hasNestedParameter(std::string parameter_name)
    {
        try
        {
            // Get all parameter names from the node
            auto all_parameters = parameters_client->list_parameters({}, 10);

            for (const auto &param_name : all_parameters.names)
            {
                if (param_name == parameter_name)
                {
                    return true;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(service_node->get_logger(), "Error while listing parameters: %s", e.what());
        }

        return false;
    }

    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr TaskGeneratorPanel::sendRequest(
        const typename rclcpp::Client<ServiceT>::SharedPtr &client,
        const typename ServiceT::Request::SharedPtr &request,
        const std::string &service_name,
        std::chrono::milliseconds cooldown)
    {
        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(10)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(service_node->get_logger(),
                             "Interrupted while waiting for the service [%s]. Exiting.",
                             service_name.c_str());
                return nullptr;
            }
            RCLCPP_INFO(service_node->get_logger(),
                        "Service [%s] not available, waiting again...", service_name.c_str());
        }

        // Send async request
        auto future = client->async_send_request(request);

        // Wait for result
        if (rclcpp::spin_until_future_complete(service_node, future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(service_node->get_logger(),
                        "Got response from service [%s]!", service_name.c_str());

            rclcpp::sleep_for(cooldown);
            return future.get();
        }
        else
        {
            RCLCPP_ERROR(service_node->get_logger(),
                         "Failed to call service [%s]!", service_name.c_str());

            rclcpp::sleep_for(cooldown);
            return nullptr;
        }
    }

} // namespace task_generator_gui