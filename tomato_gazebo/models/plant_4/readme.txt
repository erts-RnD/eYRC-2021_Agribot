          <!-- Nested model was also tried but there is some GUI bug that does not allow me to apply force to the link in case of nested model, see that approach in bak.sdf -->
                  <!-- analyse joint parameters once -->
                            <!-- The parent link is world in the examples but gazebo crashes on detach in that case! -->
          <!-- <child>tomato::t_link</child> -->
                      <!-- Can turn always_on to false if creating world plugin -->
                                  <!-- Update rate will not matter in real robot, in force torque default example (not plugin example) they have used 30, but when applying instantaneous force for testing 1000 is better as it will not cause "misses" after clicking apply force in real robot even a update rate of 1 will do as the robot will apply continuous force, we can thus simulate it taking variable time to actually cut!-->