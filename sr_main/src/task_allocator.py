#!/usr/bin/env python3

'''
Task Allocator for 8-Bot Swarm
- Monitors crate availability
- Monitors lifter idle status
- Assigns crates to idle lifters
- Tracks assignments
- Handles reassignment on failure
'''

class TaskAllocator:
    """
    Manages task allocation for multi-robot swarm.
    Uses greedy first-available assignment strategy.
    """
    
    def __init__(self, num_pairs=4, num_crates=4):
        """
        Initialize task allocator
        
        Args:
            num_pairs: Number of lifter-runner pairs (default 4)
            num_crates: Number of crates (default 4)
        """
        self.num_pairs = num_pairs
        self.num_crates = num_crates
        
        # Track crate status
        self.crates = {}
        for i in range(1, num_crates + 1):
            self.crates[f'crate_red_{i}'] = {
                'assigned': False,
                'lifter': None,
                'completed': False,
                'assignment_time': None
            }
        
        # Track lifter status
        self.lifters = {}
        for i in range(1, num_pairs + 1):
            self.lifters[f'lifter{i}'] = {
                'state': 'waiting_for_task',
                'assigned_crate': None,
                'work_start_time': None,
            }
    
    def get_idle_lifters(self):
        """Get list of lifters waiting for task assignment"""
        idle_lifters = []
        for lifter_name, lifter_info in self.lifters.items():
            if lifter_info['state'] == 'waiting_for_task':
                idle_lifters.append(lifter_name)
        return idle_lifters
    
    def get_unassigned_crates(self):
        """Get list of crates not yet assigned"""
        unassigned_crates = []
        for crate_name, crate_info in self.crates.items():
            if not crate_info['assigned'] and not crate_info['completed']:
                unassigned_crates.append(crate_name)
        return unassigned_crates
    
    def assign_task(self, lifter_name, crate_name):
        """
        Assign a crate to a lifter
        
        Returns:
            True if assignment successful, False otherwise
        """
        # Validate inputs
        if lifter_name not in self.lifters:
            return False
        if crate_name not in self.crates:
            return False
        
        lifter = self.lifters[lifter_name]
        crate = self.crates[crate_name]
        
        # Check lifter is idle
        if lifter['state'] != 'waiting_for_task':
            return False
        
        # Check crate is available
        if crate['assigned'] or crate['completed']:
            return False
        
        # Assign the task ✅
        crate['assigned'] = True
        crate['lifter'] = lifter_name
        lifter['assigned_crate'] = crate_name
        lifter['state'] = 'moving_to_crate'
        
        return True
    
    def release_task(self, lifter_name):
        """Release a task (crate delivered successfully)"""
        lifter = self.lifters[lifter_name]
        crate_name = lifter['assigned_crate']
        
        if crate_name:
            crate = self.crates[crate_name]
            crate['completed'] = True
            crate['assigned'] = False
        
        # Lifter ready for next task
        lifter['assigned_crate'] = None
        lifter['state'] = 'waiting_for_task'
    
    def reallocate_task(self, lifter_name):
        """Reallocate a task if lifter failed"""
        lifter = self.lifters[lifter_name]
        crate_name = lifter['assigned_crate']
        
        if crate_name:
            crate = self.crates[crate_name]
            crate['assigned'] = False
            crate['lifter'] = None
        
        # Lifter ready for new task
        lifter['assigned_crate'] = None
        lifter['state'] = 'waiting_for_task'
    
    def allocate_one_task(self):
        """Allocate ONE task if possible (greedy first-available)"""
        idle_lifters = self.get_idle_lifters()
        unassigned_crates = self.get_unassigned_crates()
        
        # If both available, assign first to first
        if idle_lifters and unassigned_crates:
            lifter_name = idle_lifters[0]
            crate_name = unassigned_crates[0]
            
            if self.assign_task(lifter_name, crate_name):
                return (lifter_name, crate_name)
        
        return None
    
    def allocate_all_available_tasks(self):
        """Allocate ALL available tasks at once"""
        assignments = []
        while True:
            result = self.allocate_one_task()
            if result is None:
                break
            assignments.append(result)
        return assignments
    
    def update_lifter_state(self, lifter_name, new_state):
        """Update lifter state (called from main controller)"""
        if lifter_name in self.lifters:
            self.lifters[lifter_name]['state'] = new_state
    
    def get_status(self):
        """Get current allocation status"""
        status = {
            'total_crates': self.num_crates,
            'assigned_crates': sum(1 for c in self.crates.values() if c['assigned']),
            'completed_crates': sum(1 for c in self.crates.values() if c['completed']),
            'idle_lifters': len(self.get_idle_lifters()),
            'working_lifters': self.num_pairs - len(self.get_idle_lifters()),
            'allocations': {
                lifter: self.lifters[lifter]['assigned_crate'] 
                for lifter in self.lifters if self.lifters[lifter]['assigned_crate']
            }
        }
        return status


if __name__ == '__main__':
    allocator = TaskAllocator(num_pairs=4, num_crates=4)
    print("✓ Task Allocator initialized")
    print(allocator.get_status())