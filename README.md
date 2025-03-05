# Automated Planning for Healthcare Logistics

### University of Trento - Artificial Intelligence Systems  
**Authors**: Chiara Musso, Emmanuele V. Coppola  
This project is part of the **Automated Planning** course taught by **Professor Marco Roveri**.

## Overview

This project explores **Automated Planning** in a **healthcare logistics scenario** using **PDDL/HDDL** and the **ROS2-based PlanSys2 framework**. The implementation involves **sequential planning problems** of increasing complexity, incorporating:

- **Hierarchical Task Networks (HTN)**
- **Durative actions**
- **Multi-agent coordination**

Solutions were tested using **state-of-the-art planners** such as **Fast Downward** and **Optic**, demonstrating the feasibility of automated planning in healthcare logistics and revealing key challenges and optimization opportunities.

---

## Problem Description

The project models a **structured healthcare facility** where robotic agents must:

- **Transport medical supplies**  
- **Escort patients to their assigned medical units**  

The planning environment includes:

- **Medical units** that require specific supplies.
- **A warehouse** where all supplies are initially stored.
- **Two types of robots**:  
  - **Delivery robots** for supply transportation.  
  - **Accompanying robots** for patient escort.  

The project consists of **five problems**, each increasing in complexity:

| Problem | Description |
|---------|------------|
| **1** | Basic logistics for supply delivery and patient transport. |
| **2** | Introduction of **carriers** with **load capacity constraints**. |
| **3** | Implementation of **Hierarchical Task Networks (HTN)** using **HDDL**. |
| **4** | Incorporation of **durative actions** to handle **time constraints** and **parallel execution**. |
| **5** | Integration with **PlanSys2** for real-world simulation. |

---

## Technologies & Tools

This project utilizes the following **planning frameworks and tools**:

- **PDDL (Planning Domain Definition Language)**  
- **HDDL (Hierarchical Domain Definition Language) for HTN**  
- **Fast Downward Planner** (for classical PDDL planning)  
- **PANDA Planner** (for HTN-based planning)  
- **Optic Planner** (for durative action support)  
- **PlanSys2** (ROS2 framework for task execution)  

---

## Project Structure

```
Documents/                 # Project reports and additional documentation
prob1/                     # Problem 1 implementation (basic logistics)
prob2/                     # Problem 2 implementation (carriers, constraints)
prob3/                     # Problem 3 implementation (HTN planning)
prob4/                     # Problem 4 implementation (durative actions)
prob5/                     # Problem 5 implementation (PlanSys2 integration)
README.md                  # Project documentation
assignment.pdf              # Full assignment description
```

---

## Contact

For any questions or collaboration opportunities, visit:

- **Chiara Musso** - [GitHub](https://github.com/ChiaraMuss)  
- **Emmanuele V. Coppola** - [GitHub](https://github.com/EmmanuelEngineer)  

---

