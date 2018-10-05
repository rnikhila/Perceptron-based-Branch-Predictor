// my_predictor.h
// This file contains a sample my_predictor class.
#include <iostream>
#include<cmath>
#include <math.h>

using namespace std;
class my_update : public branch_update {
public:
	unsigned int index;
};

class my_predictor : public branch_predictor {
public:
#define HISTORY_LENGTH	61
#define TABLE_BITS	15
#define MAX_ITER        12
#define Num_Perceptrons  4096
#define THRESHOLD       1.93*HISTORY_LENGTH + 14
#define DEBUG 		0

	my_update u;
	branch_info bi;
	unsigned long long int history;
	unsigned int predicted_iter;
	unsigned int BTB_miss_iter;
	unsigned char tab[1<<TABLE_BITS];
	unsigned int targets[1<<TABLE_BITS];
	unsigned int hash_val[MAX_ITER];
	int y;

	int perceptron_table[Num_Perceptrons][HISTORY_LENGTH+1];
	int bias[Num_Perceptrons];
	my_predictor (void) : history(0),predicted_iter(MAX_ITER),BTB_miss_iter(MAX_ITER),y(0) { 
		memset (perceptron_table, 0, perceptron_table[0][0]*Num_Perceptrons*(HISTORY_LENGTH+1));
		memset (bias, 0, sizeof (bias));	
		memset (tab, 0, sizeof (tab));
		memset (targets, 0, sizeof (targets));
		int i;
		for(i=0;i<MAX_ITER;i++)
		{
			hash_val[i] = rand();
		}
	}
// to update vpca and vghr after evry conditional branch
	void get_new_vpca_vghr_iter (unsigned int curr_pc, unsigned int &vpca, unsigned int &iter, unsigned long long int &vghr)
	{
		vpca = curr_pc ^ hash_val[iter];    // hashing the PC value with a random int for evry branch of indirect branch
		vghr = vghr << 1;
		vghr &= (1ULL<<HISTORY_LENGTH)-1;
		iter++;
	}
//training of perceptrons  based on the outcome and prediction 
	void update_conditional_bp_perceptron(int &y,bool taken,unsigned int curr_pc,unsigned long long int history)
	{
		int sign_y = (y > 0) ? 1 : -1;
		int sign_t = taken  ? 1 : -1;
		
		int perceptron_number = curr_pc % Num_Perceptrons;
		int weight_threshold = (1 << static_cast<int>(log2(THRESHOLD))) -1;
		if ((sign_y != sign_t) || (abs(y) <= THRESHOLD))
		{
			for (int i = 0; i < HISTORY_LENGTH+1; ++i) {
			int history_table_entry = (history & (1 << i)) ? 1 : -1;
			if(abs(perceptron_table[perceptron_number][i]) < weight_threshold)  //updating only if its less than weight threshold, it cant be more than that because the bits allocated wont be enough 
				perceptron_table[perceptron_number][i] += sign_t * history_table_entry;
			}
		}

		if(abs(bias[perceptron_number]) < weight_threshold)	  //same is followed for threshold , updating only if its less than weight threshold
	           bias[perceptron_number] += sign_t;

		update_conditional_history(taken);
	}

// updateing global history , its also updated after every conditional branch 
	void update_conditional_history(bool taken)
	{
		history <<= 1;
        	history |= taken;
        	history &= (1ULL<<HISTORY_LENGTH)-1;
	}

//predict function of perceptron - dirction predicted is updated in predicted_dir variable which is why its passed by reference
	void get_dir_pred_yout (unsigned long long int history, unsigned int curr_pc, bool &predicted_dir, int &y)
	{
		int perceptron_table_index = curr_pc % Num_Perceptrons;
		y = bias[perceptron_table_index];
		for (int i = 0; i < HISTORY_LENGTH; ++i) {
		int history_register_bit = (history & (1 << i)) ? 1 : -1;    // since its -1 for history bit 0 i.e not taken
		y += perceptron_table[perceptron_table_index][i] * history_register_bit;  // dot product of history bit and 
		}
		predicted_dir = (y > 0) ? (1) : (0); // direction predicted based on the sign of y 
	}
	
	branch_update *predict (branch_info & b) {
		bi = b;
		if(DEBUG)
		{
			cout<<  " predicted iteration initial value : " << predicted_iter << "\n";
		}
		unsigned int vpca = b.address ;
		unsigned long long int vghr = history;
		bool done = 0;
		unsigned int predicted_addr ;
		bool predicted_dir;
		unsigned int iter=0;

		if (b.br_flags & BR_INDIRECT)
		{
		while(!done)
		{
			predicted_addr = targets[vpca & ((1<<TABLE_BITS)-1)];
			get_dir_pred_yout(vghr,vpca,predicted_dir,y);
//dir_prediction

//vpc	
// target is predicted if the direction is predicted to be taken
		
			if((predicted_addr != 0)  && predicted_dir)
			{
				u.target_prediction (predicted_addr);
				done = true;
				predicted_iter = iter;
				u.direction_prediction (predicted_dir);
			}
// the loop goes on to the next branch if not predicted in the above loop
// it keeps going on until max_iteration also when there is no target in the BTB i.e BTB miss , the loop ends
			else if ((predicted_addr==0) || (iter>=MAX_ITER))
			{
				done = true;
				BTB_miss_iter = iter;
				u.target_prediction (0);
				u.direction_prediction (0);
			}
// getting the vghr and vpca for every new branch to be computed
			get_new_vpca_vghr_iter(b.address,vpca,iter,vghr);

		}
		if (iter == MAX_ITER) BTB_miss_iter = MAX_ITER -1;
		}



	if (b.br_flags & BR_CONDITIONAL) {
			u.index = ( (history ) ^ (b.address & ((1<<TABLE_BITS)-1)))& ((1<<TABLE_BITS)-1);
			u.direction_prediction (tab[u.index] >> 1);
		}
	
	if(DEBUG){	
		cout << " Indirect branch pc address: "<< bi.address <<
			" predicted iteration : " << predicted_iter <<
			" btb miss iteration: " << BTB_miss_iter <<
			" vghr : " << vghr << " vpca: "<< vpca <<
			" history : "<< history << "\n";
		}	 
		return &u;
	}

	void update (branch_update *u, bool taken, unsigned int target) 
	{
		unsigned int vpca;
                unsigned long long int vghr;
                unsigned int iter;

		
		if (bi.br_flags & BR_CONDITIONAL) 
			{
			unsigned char *c = &tab[((my_update*)u)->index];
			if (taken) {
				if (*c < 3) (*c)++;
			} else {
				if (*c > 0) (*c)--;
			}
			history <<= 1;
			history |= taken;
			history &= (1ULL<<HISTORY_LENGTH)-1;
		}

		if (bi.br_flags & BR_INDIRECT) 
		{
		vpca = bi.address ;
		vghr = history;
		iter=0;
		unsigned int predicted_addr;
		bool correct_prediction;
		correct_prediction = u->target_prediction() & (u->target_prediction() == target) ;

		if(correct_prediction)  // update loop for correct prediction
		{
		vpca = bi.address;
		  vghr = history;
		while(iter <= predicted_iter)
		{
			
			if(iter == predicted_iter)
			{
				y = 0;
				bool pred_dir;
// fetching the direction once again 
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
	// train th perceptron i.e update the weights of the each vector for if-taken	
				update_conditional_bp_perceptron(y,true,vpca,vghr);

			}

			else
			{
					y = 0;
				bool pred_dir;
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
		//similarly updating the perceptron vector for if not taken
				update_conditional_bp_perceptron(y,false,vpca,vghr);			
			}
			// getting new vpca and vghr for each conditional branch
			get_new_vpca_vghr_iter(bi.address,vpca,iter,vghr);
		}
		}
		if(!correct_prediction)
		{	
			 vghr = history;
			vpca = bi.address ;
			unsigned int target_found_iter = 0;
			bool found_crct_target = false;
			while((iter < MAX_ITER) & (!found_crct_target))
			{
				predicted_addr = targets[vpca & ((1<<TABLE_BITS)-1)];	

//correct target prediction and dir misprediction

				  if(predicted_addr == target)  
				{
					target_found_iter = iter;
					found_crct_target = true;
				}
//target mis-prediction
// BTB Miss 
				get_new_vpca_vghr_iter(bi.address,vpca,iter,vghr);
			}
			
			if(!found_crct_target)
			{
				vpca = bi.address; 
				vghr = history;
				unsigned int k =1;
				unsigned int l ;
				while(k <=  BTB_miss_iter)
				{
			//		update_conditional_bp(u, false, vpca, vghr);
				y = 0;
				bool pred_dir;
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
		
				update_conditional_bp_perceptron(y,false,vpca,vghr);
					l = k;
					l--;
					get_new_vpca_vghr_iter(bi.address, vpca, l,vghr);
					k++;
				}
				targets[vpca & ((1<<TABLE_BITS)-1)] = target;  
//				update_conditional_bp(u, true, vpca, vghr);
					y = 0;
				bool pred_dir;
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
		
				update_conditional_bp_perceptron(y,true,vpca,vghr);
			}
			else // found_crct_target
			{
 				vpca = bi.address;
                		vghr = history;
				unsigned int j = 0;
                		while(j < target_found_iter)
				{
               //     		update_conditional_bp(u, false, vpca, vghr);
				y = 0;
				bool pred_dir;
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
		
				update_conditional_bp_perceptron(y,false,vpca,vghr);
                    		get_new_vpca_vghr_iter(bi.address, vpca, j,vghr);
                		}
				y = 0;
				bool pred_dir;
				get_dir_pred_yout(vghr,vpca,pred_dir,y);
		
				update_conditional_bp_perceptron(y,true,vpca,vghr);
//				update_conditional_bp(u, true, vpca, vghr);

			}
			}//if !correct_prediction
if(DEBUG)
{
cout << "Updated History: "<< history <<" Any branch taken: "<< taken <<" target: " << target << "\n";
}

		}	//indirect loop end
		BTB_miss_iter = MAX_ITER;
		predicted_iter = MAX_ITER;
	}//update loop end
};
