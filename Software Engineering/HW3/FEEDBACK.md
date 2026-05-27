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
- **Overly Coupled Architectures**: Some presentations showed class diagrams where every single class had a direct association to every other class. I felt this created tight coupling that would make maintaining or testing the code very difficult in a real programming environment.
- **Underspecified Interfaces**: A few teams presented methods on their diagrams without specifying input parameters or return types. During the implementation phase, this leaves too much ambiguity. I paid extra attention to clearly defining my python method interfaces to avoid this issue.
