def pick_up_the_fruit():
    open_claw()
    move_robot_forwards_until_close_enough_to_pick_up()
    close_claw()
    move_robot_backwards()
    lower_claw()