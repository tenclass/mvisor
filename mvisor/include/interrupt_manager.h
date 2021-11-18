#ifndef _MVISOR_CORE_INTERRUPT_MANAGER_H
#define _MVISOR_CORE_INTERRUPT_MANAGER_H


class Machine;
class InterruptManager {
 public:
  InterruptManager(Machine* machine);
  ~InterruptManager();

  
 private:
  void InitializeRouting();


  Machine* machine_;
};

#endif // _MVISOR_CORE_INTERRUPT_MANAGER_H
