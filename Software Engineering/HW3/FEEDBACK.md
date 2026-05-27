# Course & Teamwork Feedback - Homework 3

In this document, I present my personal reflection on the modeling and collaborative design process conducted in class, as well as my observations on the presentations given by other teams.

## Feedback on Teamwork in Class

### What I Liked
- **Collaborative Problem Solving**: I thoroughly enjoyed discussing how to partition a complex mechanical entity like a Mars Rover into distinct software classes. Collaborating with peers allowed me to see that architectural decisions are rarely simple.
- **Architectural Brainstorming**: Whiteboarding the connections between the `Sensors`, `Lab`, and `Actuators` helped me understand the concept of aggregation and shared dependencies (such as the `Camera` which acts as both an optical sensor and an actuator requiring rotation commands).
- **Consensus Building**: Aligning on a common state machine helped me appreciate how design guidelines prevent implementation conflicts later in a project.

### What I Disliked
- **Scope Creep**: During our initial discussions, there was a tendency to add excessive details to the class diagram that were not strictly necessary for the core task. I had to advocate for simplifying the modeling to keep it practical and functional.
- **Inefficient Time Management**: Group discussions occasionally stalled on minor naming conventions (e.g., whether to use `Tx/Rx` or `Antenna`), which delayed the drawing of our sequence diagrams. I realized that keeping a tight timeline is essential in group projects.

---

## Feedback on Presentations of Other Teams

### Positive Observations
- **Clear Sequence Diagrams**: One of the teams presented a highly structured sequence diagram for their communication unit that clearly separated synchronous acknowledgments from asynchronous replies. I found their notation very clear and decided to use a similar clean separation in my python simulation flow.
- **Robust Exception States**: Another team spent considerable time designing exception transitions within their state machine (e.g., what happens if a self-diagnosis fail is encountered mid-mission). This was an excellent addition that reminded me to build explicit error transitions into my class structures.

### Critique and Areas for Improvement
- **Missing Diagnostic Helper Methods**: One of the critiques received on my presentation was that our class diagram was slightly underspecified regarding diagnostic coordination, as it lacked helper methods to verify the communication paths of individual sub-systems (such as explicitly connecting to and querying actuators, PMU, and sensors). To address this feedback in the implementation, I added minor internal check methods (like `connect_move` and `ask_move`) inside the respective components. This allowed me to execute the full sequence diagram diagnostic routines cleanly without altering the core class structures or introducing unrelated APIs.
