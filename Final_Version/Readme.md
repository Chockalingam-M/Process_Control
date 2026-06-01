__Adaptive LMS Filter Implementation in Simulation__

This project represents my 10-week development process in Process Control, focused on __implementing and optimizing an Adaptive LMS (Least Mean Square) Filter using Verilog HDL__.

The project began with a basic LMS filter architecture developed without optimization to understand the internal working of adaptive filtering, including coefficient updating, error calculation, and signal convergence. 
After verifying the functionality, the design was progressively improved using parallel processing and pipelined architecture to reduce computation delay and improve throughput.

The implementation was first tested using a 16-tap LMS filter and later extended to a 64-tap LMS filter for improved adaptive performance. The architecture includes:
  Parallel multipliers
  Multi-stage registered adder tree
  Pipeline stages for high-speed computation
  Fixed-point arithmetic operations
  
The LMS filter updates its coefficients using the adaptive learning equation:
 __w(n+1)=w(n)+μe(n)x(n)__
where the error signal is calculated as:
  __e(n)=d(n)−y(n)__

First, we built the FPGA analysis. The generated code simulates the frequency noise of 200Hz and stores the output in CSV format. For further analysis, we used MATLAB to find the error and create a convergence graph for observation.  We then simulated the same code on the Microncontroller and used Looger.py to push the parameters to VSCode.  From there, we converted and stored them in a CSV file. Finally, we imported the same CSV file into MATLAB for further analysis. 

Simulation outputs were exported into CSV format and analyzed in MATLAB to study:
  Error convergence
  RMS and Mean Square Error
  Signal waveform behavior
  Overall filter performance

From the analysis, it was observed that the FPGA implementation achieved significantly better performance. 
