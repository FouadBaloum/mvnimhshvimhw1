//
// Created by robotic on 07/05/2023.
//

/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include "cmath"

class BTB {
public:
    unsigned btbSize;
    unsigned historySize;
    unsigned tagSize;
    unsigned fsmState;
    uint32_t* target_pc;
    uint32_t* tag;
    bool* valid;
    int related_bits_btb;
    unsigned branches_num;
    unsigned flush_num;
    BTB(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) : btbSize(btbSize),
                historySize(historySize), tagSize(tagSize), fsmState(fsmState),target_pc(new uint32_t [btbSize]),
                tag(new uint32_t[btbSize]),valid(new bool[btbSize]),branches_num(0),flush_num(0) {
        related_bits_btb=(int)log2(btbSize);
        for (int i = 0; i < (int) btbSize; ++i) {
            tag[i]=0;
            target_pc[i]=0;
            valid[i]= false;

        }
    }
    ~BTB() = default;
    virtual bool  predict(uint32_t pc, uint32_t *dst) =0;
    virtual void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) = 0;
    virtual void  BP_GetStats(SIM_stats *curStats) = 0;
};

BTB * btb;

class BTB_GH : public BTB{
public:
    int history;
    BTB_GH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState)
                : BTB(btbSize, historySize, tagSize, fsmState) , history(0){}
    ~BTB_GH() =default;


};
class BTB_LH : public BTB{
public:
    int* history;
    BTB_LH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState)
            : BTB(btbSize, historySize, tagSize, fsmState) , history(new int[btbSize]){
        for (int i = 0; i < (int) btbSize; ++i) {
            history[i]=0;
        }
    }
    ~BTB_LH() =default;
};

class BTB_GH_GT : public BTB_GH{
public:
    unsigned * global_table;
    int Shared;
    BTB_GH_GT(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int Shared) :BTB_GH(btbSize,historySize,tagSize,fsmState)
    ,global_table(new unsigned [(int)pow(2,historySize)]), Shared(Shared){
        for (int i = 0; i < (int)pow(2,historySize); ++i) {
            global_table[i]=fsmState;
        }
    }
    ~BTB_GH_GT()=default;

     bool  predict(uint32_t pc, uint32_t *dst) override {
         uint32_t index=(pc/4)%(int)pow(2,related_bits_btb);
         uint32_t tmp_tag=((pc/(int)pow(2,2+related_bits_btb))%(int)pow(2,tagSize));
         if (tag[index]!=tmp_tag || !valid[index]){
             *dst=pc+4;
             return false;
         }
        uint32_t index_in_GT;

         if (Shared==0){
             index_in_GT=history;
         }
         if (Shared==1){
             index_in_GT= (history ^ (pc/4))%(int)pow(2,historySize);
         }
         if (Shared==2){
             index_in_GT= (history ^ (pc/(int)pow(2,16)))%(int)pow(2,historySize);
         }
         if (global_table[index_in_GT]<2){
             *dst=pc+4;
             return false;
         }
         *dst=target_pc[index];
         return true;
     }

     void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override{
         uint32_t index=(pc/4)%(int)pow(2,related_bits_btb);
         uint32_t tmp_tag=((pc/(int)pow(2,2+related_bits_btb))%(int)pow(2,tagSize));
         branches_num++;////askk
         uint32_t index_in_GT;

         if (Shared==0){
             index_in_GT=history;
         }
         if (Shared==1){
             index_in_GT= (history ^ (pc/4))%(int)pow(2,historySize);
         }
         if (Shared==2){
             index_in_GT= (history ^ (pc/(int)pow(2,16)))%(int)pow(2,historySize);
         }
         if (!valid[index] || tag[index]!=tmp_tag){
             valid[index]= true;
             tag[index]=tmp_tag;
             target_pc[index]= targetPc;
             if (taken){
                 flush_num++;
                 if (global_table[index_in_GT]<3){
                     global_table[index_in_GT]++;
                 }
                 history=(history*2+1) % (int) pow(2, historySize);
             }
             else{

                 if (global_table[index_in_GT]>0){
                     global_table[index_in_GT]--;
                 }
                 history=(history*2) % (int) pow(2, historySize);
             }
             return;
         }

         if (!taken){
             if (pred_dst != pc+4)
                 flush_num++;
             if (global_table[index_in_GT]>0)
                 global_table[index_in_GT]--;
             history=(history*2)% (int) pow(2, historySize);
             target_pc[index]= targetPc;

             return;
         }
         if (pred_dst != targetPc )
             flush_num++;
         if (global_table[index_in_GT]<3)
             global_table[index_in_GT]++;
         history=(history*2+1)% (int) pow(2, historySize);
         target_pc[index]= targetPc;

     }

     void  BP_GetStats(SIM_stats *curStats) override{
         curStats->flush_num=flush_num;
         curStats->br_num=branches_num;
         curStats->size=(unsigned)btbSize*(1+tagSize+30)+(unsigned )(2*pow(2,historySize)+historySize);

     }

};

class BTB_GH_LT : public BTB_GH{
public:
    unsigned ** local_tables;
    BTB_GH_LT(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) :BTB_GH(btbSize,historySize,tagSize,fsmState)
            ,local_tables(new unsigned* [(int)btbSize]){
        for(unsigned i=0; i<btbSize; i++){
            local_tables[i]= new unsigned[(int)pow(2, historySize)]{fsmState};
            for (int j = 0; j < (int)pow(2,historySize); ++j) {
                local_tables[i][j]=fsmState;
            }
        }
    }
    ~BTB_GH_LT()=default;
    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t index=(pc/4)%(int)pow(2,related_bits_btb);
        uint32_t tmp_tag=((pc/(int)pow(2,2+related_bits_btb))%(int)pow(2,tagSize));
        if (tag[index]!=tmp_tag || !valid[index]){
            *dst=pc+4;
            return false;
        }
        if(local_tables[index][history] < 2){
            *dst=pc+4;
            return false;
        }
        *dst=target_pc[index];
        return true;
    }
    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override{
        uint32_t index=(pc/4)%(int)pow(2,related_bits_btb);
        uint32_t tmp_tag=((pc/(int)pow(2,2+related_bits_btb))%(int)pow(2,tagSize));
        branches_num++;////askk
        if (!valid[index] || tag[index]!=tmp_tag){
            for (int i = 0; i < (int)pow(2,historySize); ++i) {
                local_tables[index][i]=fsmState;
            }
            valid[index]= true;
            tag[index]=tmp_tag;
            target_pc[index]= targetPc;
            if (taken){
                flush_num++;
                local_tables[index][history] = fsmState;
                if (local_tables[index][history]<3){
                    local_tables[index][history]++;
                }
                history=(history*2+1)%(int)pow(2,historySize); /////ask
            }
            else{
                local_tables[index][history] = fsmState;
                if (local_tables[index][history]>0){
                    local_tables[index][history]--;
                }
                history=(history*2)%(int)pow(2,historySize);
            }
            return;
        }

        if (!taken){
            if (pred_dst != pc+4)
                flush_num++;
            if (local_tables[index][history]>0)
                local_tables[index][history]--;
            history=(history*2)%(int)pow(2,historySize);
            target_pc[index]= targetPc;

            return;
        }
        if (pred_dst != targetPc)
            flush_num++;
        if (local_tables[index][history]<3)
            local_tables[index][history]++;
        history=(history*2+1)%(int)pow(2,historySize);
        target_pc[index]= targetPc;

    }
    void  BP_GetStats(SIM_stats *curStats) override{
        curStats->flush_num=flush_num;
        curStats->br_num=branches_num;
        curStats->size=(unsigned)btbSize*(1+tagSize+30+2*pow(2,historySize))+(unsigned )historySize;
    }
};

class BTB_LH_LT : public BTB_LH {
public:
    unsigned **local_tables;

    BTB_LH_LT(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) : BTB_LH(btbSize,
                                                                                                    historySize,
                                                                                                    tagSize, fsmState),
                                                                                             local_tables(
                                                                                                     new unsigned *[(int) btbSize]) {
        for (unsigned i = 0; i < btbSize; i++) {
            local_tables[i] = new unsigned[(int) pow(2, historySize)]{fsmState};
            for (int j = 0; j < (int)pow(2,historySize); ++j) {
                local_tables[i][j]=fsmState;
            }
        }
    }

    ~BTB_LH_LT() = default;

    bool predict(uint32_t pc, uint32_t *dst) override {
        uint32_t index = (pc / 4) % (int) pow(2, related_bits_btb);
        uint32_t tmp_tag = ((pc / (int) pow(2, 2 + related_bits_btb)) % (int) pow(2, tagSize));
        if (tag[index] != tmp_tag || !valid[index]) {
            *dst = pc + 4;
            return false;
        }
        if (local_tables[index][history[index]] < 2) {
            *dst = pc + 4;
            return false;
        }
        *dst = target_pc[index];
        return true;
    }

    void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t index = (pc / 4) % (int) pow(2, related_bits_btb);
        uint32_t tmp_tag = ((pc / (int) pow(2, 2 + related_bits_btb)) % (int) pow(2, tagSize));
        branches_num++;////askk
        if (!valid[index] || tag[index] != tmp_tag) {
            history[index]=0;
            for (int i = 0; i < (int)pow(2,historySize); ++i) {
                local_tables[index][i]=fsmState;
            }
            valid[index] = true;
            tag[index] = tmp_tag;
            target_pc[index] = targetPc;
            if (taken) {
                flush_num++;
                local_tables[index][history[index]] = fsmState;
                if (local_tables[index][history[index]] < 3) {
                    local_tables[index][history[index]]++;
                }
                history[index] = (history[index] * 2 + 1) % (int) pow(2, historySize); /////ask
            } else {
                local_tables[index][history[index]] = fsmState;
                if (local_tables[index][history[index]] > 0) {
                    local_tables[index][history[index]]--;
                }
                history[index] = (history[index] * 2) % (int) pow(2, historySize);
            }
            return;
        }

        if (!taken) {
            if (pred_dst != pc + 4)
                flush_num++;
            if (local_tables[index][history[index]] > 0)
                local_tables[index][history[index]]--;
            history[index] = (history[index] * 2) % (int) pow(2, historySize);
            target_pc[index] = targetPc;

            return;
        }
        if (pred_dst != targetPc)
            flush_num++;
        if (local_tables[index][history[index]] < 3)
            local_tables[index][history[index]]++;
        history[index] = (history[index] * 2 + 1) % (int) pow(2, historySize);
        target_pc[index] = targetPc;

    }

    void BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = flush_num;
        curStats->br_num = branches_num;
        curStats->size=(unsigned)btbSize*(1+tagSize+30+2*pow(2,historySize)+historySize);
    }
};

class BTB_LH_GT : public BTB_LH {
public:
    unsigned *global_table;
    int Shared;

    BTB_LH_GT(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int Shared) : BTB_LH(btbSize,historySize,tagSize, fsmState),
    global_table(new unsigned[(int) pow(2,historySize)]{fsmState}), Shared(Shared){
        for (int i = 0; i < (int)pow(2,historySize); ++i) {
            global_table[i]=fsmState;
        }
    }


    ~BTB_LH_GT() = default;

    bool predict(uint32_t pc, uint32_t *dst) override {
        uint32_t index = (pc / 4) % (int) pow(2, related_bits_btb);
        uint32_t tmp_tag = ((pc / (int) pow(2, 2 + related_bits_btb)) % (int) pow(2, tagSize));
        if (tag[index] != tmp_tag || !valid[index]) {
            *dst = pc + 4;
            return false;
        }
        uint32_t index_in_GT;

        if (Shared == 0) {
            index_in_GT = history[index];
        }
        if (Shared == 1) {
            index_in_GT = (history[index] ^ (pc / 4)) % (int) pow(2, historySize);
        }
        if (Shared == 2) {
            index_in_GT = (history[index] ^ (pc / (int) pow(2, 16))) % (int) pow(2, historySize);
        }
        if (global_table[index_in_GT] < 2) {
            *dst = pc + 4;
            return false;
        }
        *dst = target_pc[index];
        return true;
    }

    void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t index = (pc / 4) % (int) pow(2, related_bits_btb);
        uint32_t tmp_tag = ((pc / (int) pow(2, 2 + related_bits_btb)) % (int) pow(2, tagSize));
        branches_num++;////askk
        uint32_t index_in_GT;
        if (!valid[index] || tag[index] != tmp_tag) {
            history[index] = 0;
        }
        if (Shared == 0) {
            index_in_GT = history[index];
        }
        if (Shared == 1) {
            index_in_GT = (history[index] ^ (pc / 4)) % (int) pow(2, historySize);
        }
        if (Shared == 2) {
            index_in_GT = (history[index] ^ (pc / (int) pow(2, 16))) % (int) pow(2, historySize);
        }
        if (!valid[index] || tag[index] != tmp_tag) {
            valid[index] = true;
            tag[index] = tmp_tag;
            target_pc[index] = targetPc;
            if (taken) {
                flush_num++;
                if (global_table[index_in_GT] < 3) {
                    global_table[index_in_GT]++;
                }
                history[index] = (history[index] * 2 + 1)% (int) pow(2, historySize);
            } else {
                if (global_table[index_in_GT] > 0) {
                    global_table[index_in_GT]--;
                }
                history[index] = (history[index] * 2)% (int) pow(2, historySize);
            }
            return;
        }

        if (!taken) {
            if (pred_dst != pc + 4)
                flush_num++;
            if (global_table[index_in_GT] > 0)
                global_table[index_in_GT]--;
            history[index] = (history[index] * 2)% (int) pow(2, historySize);
            target_pc[index] = targetPc;

            return;
        }
        if (pred_dst != targetPc)
            flush_num++;
        if (global_table[index_in_GT] < 3)
            global_table[index_in_GT]++;
        history[index] = (history[index] * 2 + 1)% (int) pow(2, historySize);
        target_pc[index] = targetPc;

    }

    void BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = flush_num;
        curStats->br_num = branches_num;
        curStats->size=(unsigned)btbSize*(1+tagSize+30+historySize)+(unsigned )2*pow(2,historySize);

    }
};


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared){
    if(isGlobalHist && isGlobalTable){
        btb = new BTB_GH_GT(btbSize,historySize,tagSize,fsmState, Shared);
    }
    else if(!isGlobalHist && isGlobalTable){
        btb = new BTB_LH_GT(btbSize,historySize,tagSize,fsmState, Shared);
    }
    else if(isGlobalHist && !isGlobalTable){
        btb = new BTB_GH_LT(btbSize,historySize,tagSize,fsmState);
    }
    else if(!isGlobalHist && !isGlobalTable){
        btb = new BTB_LH_LT(btbSize,historySize,tagSize,fsmState);
    }
    if(btb){
        return 0;
    }
    return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
    return btb->predict(pc,dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    return btb->update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
    return btb->BP_GetStats(curStats);
}

