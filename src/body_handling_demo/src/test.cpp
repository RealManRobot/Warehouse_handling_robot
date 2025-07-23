bool RosInterface::callStepCtrlAction(const StepsCtrlParameter &parameter,
                                      RosCommonCB callback) {
  if (step_ctrl_action_ == nullptr)
    step_ctrl_action_ =
        std::make_shared<StepCtrlAction>(*n_, "cmd_vel_control");

  if (!step_ctrl_action_.get()->waitForServer(ros::Duration(3))) {
    if (ros_msg_callback_ != nullptr)
      ros_msg_callback_("action server is invalid,action:cmd_vel_control");
    return false;
  }

  if (parameter.action == ControlAction::kCancel) {
    step_ctrl_action_.get()->cancelGoal();  //取消任务

    if (ros_msg_callback_ != nullptr)
      ros_msg_callback_("action canceled,action:cmd_vel_control");
  } else {
    if (parameter.action == ControlAction::kExecute && parameter.step.empty()) {
      if (ros_msg_callback_ != nullptr)
        ros_msg_callback_(
            "action failed,steps_size is empty,action:cmd_vel_control");
      return false;
    }

    woosh_msgs::StepControlGoal goal;
    goal.useAvoid = !parameter.avoid;
    for (const StepsCtrlParameter::StepCtrl &step : parameter.step) {
      woosh_msgs::StepControl step_control;
      step_control.data = step.value;
      step_control.speed = step.speed;
      step_control.angle = step.angle;
      step_control.executeMode = static_cast<uint8_t>(step.mode);
      goal.stepControl.push_back(step_control);
    }
    goal.mode = static_cast<uint8_t>(parameter.action);

    step_ctrl_action_.get()->sendGoal(
        goal,
        boost::bind(&RosInterface::stepCtrlResultCB, this, _1, _2, callback),
        []() {},
        boost::bind(&RosInterface::stepCtrlFeedbackCB, this, _1, callback));

    if (ros_msg_callback_ != nullptr)
      ros_msg_callback_("send goal,action:woosh_move_avoid");
  }

  return true;
}