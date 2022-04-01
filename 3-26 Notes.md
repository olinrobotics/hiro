# Collaborative Robot
## ExPO Target 
**Proof of Concept** of being able to collaboratively assemble a device with a robot

### Minimum Viable Product
Robot senses a single movement (wave) and does something arbitrary in response

## Division of Tasks
### Anmol, Merwan, Krishna -> Path planning
* Setting up ROS/dependencies
* Getting familiar with charting out a path
* Identify a purpose
	* Reach and grab an object in the scene (CV & path planning)
  
### Ayush, Raiyyan, Mahima -> Vision (DeepPose)
* Get existing deeppose checkpoint running
- Demo of one of you waving your arms
- Combing through the output
	- Finding arms (left forearm, left upper arm, etc.) and profile the motion
- Finding some identifiable pattern in a single wave

## Action Items
**For next meeting**
**Vision** -> Have deeppose running -> Have output & insights on it
**Path Planning** -> Setting up ROS -> Getting arm to move to a single point -> *Detecting scene objects & grab it (simulated in Gazebo )*