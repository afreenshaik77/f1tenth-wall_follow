# Lab 3: Wall Following

## I. Learning Goals

- PID Controllers
- Driving the car autonomously via Wall Following

## II. Review of PID in the time domain

A PID controller is a way to maintain certain parameters of a system around a specified set point. PID controllers are used in a variety of applications requiring closed-loop control, such as in the VESC speed controller on your car.

The general equation for a PID controller in the time domain, as discussed in lecture, is as follows:

$$ u(t)=K_{p}e(t)+K_{i}\int_{0}^{t}e(t^{\prime})dt^{\prime}+K_{d}\frac{d}{dt}(e(t)) $$

Here, $K_p$, $K_i$, and $K_d$ are constants that determine how much weight each of the three components (proportional, integral, derivative) contribute to the control output $u(t)$. $u(t)$ in our case is the steering angle we want the car to drive at. The error term $e(t)$ is the difference between the set point and the parameter we want to maintain around that set point.

## III. Wall Following

In the context of our car, the desired distance to the wall should be our set point for our controller, which means our error is the difference between the desired and actual distance to the wall. This raises an important question: how do we measure the distance to the wall, and at what point in time? One option would simply be to consider the distance to the right wall at the current time $t$ (let's call it $D_t$). Let's consider a generic orientation of the car with respect to the right wall and suppose the angle between the car's x-axis and the axis in the direction along the wall is denoted by $\alpha$. We will obtain two laser scans (distances) to the wall:
one 90 degrees to the right of the car's x-axis (beam b in the figure), and one (beam a) at an angle $\theta$ ( $0<\theta\leq70$ degrees) to the first beam. Suppose these two laser scans return distances a and b, respectively.

![fig1](img/wall_following_lab_figure_1.png)

*Figure 1: Distance and orientation of the car relative to the wall*

Using the two distances $a$ and $b$ from the laser scan, the angle $\theta$ between the laser scans, and some trigonometry, we can express $\alpha$ as

$$ \alpha=\mbox{tan}^{-1}\left(\frac{a\mbox{cos}(\theta)-b}{a\mbox{sin}(\theta)}\right) $$

We can then express $D_t$ as 

$$ D_t=b\mbox{cos}(\alpha) $$

to get the current distance between the car and the right wall. What's our error term $e(t)$, then? It's simply the difference between the desired distance and actual distance! For example, if our desired distance is 1 meter from the wall, then $e(t)$ becomes $1-D_t$.
	
However, we have a problem on our hands. Remember that this is a race: your car will be traveling at a high speed and therefore will have a non-instantaneous response to whatever speed and servo control you give to it. If we simply use the current distance to the wall, we might end up turning too late, and the car may crash. Therefore, we must look to the future and project the car ahead by a certain lookahead distance (let's call it $L$). Our new distance $D_{t+1}$ will then be

$$D_{t+1}=D_t+L\mbox{sin}(\alpha)$$

![fig1](img/wall_following_lab_figure_2.png)

*Figure 2: Finding the future distance from the car to the wall*

We're almost there. Our control algorithm gives us a steering angle for the VESC, but we would also like to slow the car down around corners for safety. We can compute the speed in a step-like fashion based on the steering angle, or equivalently the calculated error, so that as the angle exceeds progressively larger amounts, the speed is cut in discrete increments. For this lab, a good starting point for the speed control algorithm is:

- If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
- If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
- Otherwise, the speed should be 0.5 meters per second.

So, in summary, here's what we need to do:

1. Obtain two laser scans (distances) a and b.
2. Use the distances a and b to calculate the angle $\alpha$ between the car's $x$-axis and the right wall.
3. Use $\alpha$ to find the current distance $D_t$ to the car, and then $\alpha$ and $D_t$ to find the estimated future distance $D_{t+1}$ to the wall.
4. Run $D_{t+1}$ through the PID algorithm described above to get a steering angle.
5. Use the steering angle you computed in the previous step to compute a safe driving speed.
6. Publish the steering angle and driving speed to the `/drive` topic in simulation.

## IV. Implementation

Implement wall following to make the car drive autonomously around the Levine Hall map. Follow the inner walls of Levine. Which means follow left if the car is going counter-clockwise in the loop. (The first race we run will be counter-clockwise). You can implement this node in either C++ or Python.

Test in the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-jazzy) simulator on the `levine_blocked` map (`map_path: 'maps/levine_blocked'` in `config/sim.yaml`): it is the Levine loop with its doorways sealed, so the car cannot wander out. The autograder drives your node around a copy of that map with the two facing door recesses halfway down the west hallway walled off, because a two-beam wall follower steers into them; everything else is identical. The grading map ships with the simulator as `maps/levine_fully_blocked`; set `map_path: 'maps/levine_fully_blocked'` in `config/sim.yaml` to test on it. Both maps come with a centerline, so the simulator counts your laps (`/ego_racecar/lap_count`, and a "completed lap" line in the bridge log).

Your node must subscribe to `/scan` and publish `AckermannDriveStamped` on `/drive`, and it must work when started with a plain `ros2 run wall_follow <executable>`: the autograder passes no parameter file, so bake your tuned gains, desired distance and speeds into the node's defaults.

## V. Deliverables and Submission

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `wall_follow` package and directly commit and push to the repo Classroom 50 created for you. Your committed code should start and run in simulation smoothly, this includes building as a package with any dependencies included as part of the ``package.xml``.

**Deliverable 2**: Make a screen cast of running your wall following node in the simulation: at least one full counter-clockwise lap of Levine, following the inner wall. Upload your video to YouTube (unlisted) or Google Drive — for Drive, set sharing to **"Anyone with the link can view"** or we cannot grade it — and include the link in **`SUBMISSION.md`**.

### Submitting

We'll be using Classroom 50 throughout the semester to manage submissions for lab assignments. You can commit and push your work as often as you need, but a plain push does **not** count as a submission. When you're ready to submit, push a tag named `submission`:

```bash
git push                            # your commits
git tag submission
git push origin submission          # this triggers the autograder
```

The autograder builds your package, probes your controller, and drives it around Levine in the simulator, then posts your score as a **Release** on your repo (check the Releases page or the commit's status check a few minutes after you tag). To resubmit, move the tag to a new commit:

```bash
git tag -f submission
git push --force origin submission
```
The last ``submission`` push before the deadline is counted as your final submission and its grade will be your lab's grade.

**The autograder finds your work by name.** Use the names the deliverables specify: package `wall_follow` with an executable it can start with `ros2 run wall_follow <executable>` (the skeleton's `wall_follow_node`), subscribing `/scan` and publishing `/drive`. Otherwise, the autograder will not be able to grade your work and your submission may get the wrong grade.

## VI: Grading Rubric

- Compilation: **10** Points (autograded)
- Implemented PID: **40** Points (autograded without the simulator: your node is fed scans of a straight corridor with the car too close to one wall, too close to the other, and turned toward each wall, and must steer the right way in each case — whichever wall you follow)
- Tuned PID: **40** Points (autograded in simulation: one counter-clockwise lap of the grading map, starting on the south hallway heading east, without touching a wall; a run that ends early earns partial credit for the fraction of the loop covered)
- Video: **10** Points (TA-graded from the link in `SUBMISSION.md`: YouTube unlisted, or Google Drive shared as "Anyone with the link can view")
