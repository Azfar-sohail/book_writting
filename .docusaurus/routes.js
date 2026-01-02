import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/book-writting/',
    component: ComponentCreator('/book-writting/', 'c0f'),
    routes: [
      {
        path: '/book-writting/',
        component: ComponentCreator('/book-writting/', 'a4c'),
        routes: [
          {
            path: '/book-writting/',
            component: ComponentCreator('/book-writting/', '99a'),
            routes: [
              {
                path: '/book-writting/part1/',
                component: ComponentCreator('/book-writting/part1/', 'c19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/ai_meets_the_physical_world',
                component: ComponentCreator('/book-writting/part1/ai_meets_the_physical_world', 'c7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/basics_of_robotics_systems',
                component: ComponentCreator('/book-writting/part1/basics_of_robotics_systems', '402'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/introduction_to_ros2',
                component: ComponentCreator('/book-writting/part1/introduction_to_ros2', 'd7f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/nodes_topics_and_messages',
                component: ComponentCreator('/book-writting/part1/nodes_topics_and_messages', 'd6c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/services_actions_and_parameters',
                component: ComponentCreator('/book-writting/part1/services_actions_and_parameters', '9e4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/simulation_before_reality',
                component: ComponentCreator('/book-writting/part1/simulation_before_reality', 'bad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/software_stack_for_humanoid_robots',
                component: ComponentCreator('/book-writting/part1/software_stack_for_humanoid_robots', 'eba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/urdf_robot_description',
                component: ComponentCreator('/book-writting/part1/urdf_robot_description', '0d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/what_is_physical_ai',
                component: ComponentCreator('/book-writting/part1/what_is_physical_ai', '371'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part1/writing_ros2_nodes_with_rclpy',
                component: ComponentCreator('/book-writting/part1/writing_ros2_nodes_with_rclpy', 'b65'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/',
                component: ComponentCreator('/book-writting/part2/', 'fd6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/autonomous_decision_making',
                component: ComponentCreator('/book-writting/part2/autonomous_decision_making', 'f8f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/capstone_the_autonomous_humanoid',
                component: ComponentCreator('/book-writting/part2/capstone_the_autonomous_humanoid', 'c5c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/gazebo_physics_simulation',
                component: ComponentCreator('/book-writting/part2/gazebo_physics_simulation', 'c51'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/isaac_ros_navigation_nav2',
                component: ComponentCreator('/book-writting/part2/isaac_ros_navigation_nav2', '75d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/language_to_action_planning',
                component: ComponentCreator('/book-writting/part2/language_to_action_planning', '91f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/nvidia_isaac_sim',
                component: ComponentCreator('/book-writting/part2/nvidia_isaac_sim', 'bb6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/sensors_in_simulation',
                component: ComponentCreator('/book-writting/part2/sensors_in_simulation', 'fb0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/unity_for_human_robot_interaction',
                component: ComponentCreator('/book-writting/part2/unity_for_human_robot_interaction', 'e85'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/voice_to_text_with_whisper',
                component: ComponentCreator('/book-writting/part2/voice_to_text_with_whisper', 'c76'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/part2/what_is_vision_language_action',
                component: ComponentCreator('/book-writting/part2/what_is_vision_language_action', '99c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/book-writting/rag_personalization_translation',
                component: ComponentCreator('/book-writting/rag_personalization_translation', '346'),
                exact: true
              },
              {
                path: '/book-writting/',
                component: ComponentCreator('/book-writting/', '6e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
