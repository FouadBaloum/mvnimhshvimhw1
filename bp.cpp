/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */
#include "bp_api.h"
#include <vector>

int power2tox( int x){
    int res=1;
    for(int i=0;i<x;i++){
        res*=2;
    }
    return res;
}
int log2tox(int x){
    int res=0;
    while (x > 1) {
        x /= 2;
        res++;
    }
    return res;
}

class BTB {
public:
    unsigned btbSize;
    unsigned historySize;
    unsigned tagSize;
    unsigned fsmState;
    std::vector<bool> valid;
    std::vector<uint32_t> targetpc;
    std::vector<uint32_t> tag;
    int bits_for_btb;
    int bitstotag;
    SIM_stats mystats;
    int bitstotagpower;
    int tagpower;
    int histpower;
    BTB(unsigned btbsize, unsigned historysize, unsigned tagsize, unsigned fsmstate):btbSize(btbsize),
        historySize(historysize), tagSize(tagsize), fsmState(fsmstate), valid(btbsize, false),targetpc(btbsize, 0),
        tag(btbsize, 0){
        bits_for_btb = log2tox(btbsize);
        bitstotag = 2+bits_for_btb;
        bitstotagpower = power2tox(bitstotag);
        tagpower = power2tox(tagSize);
        histpower = power2tox(historySize);
        mystats = {0,0,0};
    }
    ~BTB() = default;
    virtual bool  predict(uint32_t pc, uint32_t *dst) =0;
    virtual void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) = 0;
    virtual void  BP_GetStats(SIM_stats *curStats) = 0;
};

class BTB_GT_LH : public BTB {
public:
    std::vector<unsigned> globaltable;
    int shared;
    std::vector<int> historyvec;
    BTB_GT_LH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,int isShare) :
    BTB(btbSize, historySize, tagSize, fsmState),globaltable(histpower,fsmState),shared(isShare),historyvec(btbSize,0){}
    ~BTB_GT_LH() =default;

    uint32_t  sharetype(uint32_t pc, int shared,uint32_t myindex){
        if (shared==0){
            return historyvec[myindex];
        }
        if (shared==1){
            return ((historyvec[myindex] ^ (pc/4))%histpower);
        }
        return ((historyvec[myindex] ^ (pc/ power2tox(16)))%histpower);
    }

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        if (tag[myindex]!=mytag || !valid[myindex]){
            *dst=pc+4;
            return false;
        }
        uint32_t indexingt = sharetype(pc,shared,myindex);
        if (globaltable[indexingt] < 2) {
            *dst = pc + 4;
            return false;
        }
        *dst=targetpc[myindex];
        return true;
    }

    void updatetableup(uint32_t indexingt, uint32_t myindex){
        if (globaltable[indexingt]<3)
            globaltable[indexingt]++;
        historyvec[myindex]=((historyvec[myindex]*2+1)%histpower);
    }
    void updatetabledown(uint32_t indexingt, uint32_t myindex){
        if (globaltable[indexingt]>0)
            globaltable[indexingt]--;
        historyvec[myindex]=((historyvec[myindex]*2)%histpower);
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        if (!valid[myindex] || tag[myindex] != mytag) {
            historyvec[myindex] = 0;
        }
        uint32_t indexingt = sharetype(pc,shared,myindex);
        if (!valid[myindex] || tag[myindex]!=mytag){
            valid[myindex]= true;
            targetpc[myindex]= targetPc;
            tag[myindex]=mytag;
            if (!taken)
                updatetabledown(indexingt,myindex);
            else{
                mystats.flush_num++;
                updatetableup(indexingt,myindex);
            }
            return;
        }
        if (taken){
            updatetableup(indexingt,myindex);
            targetpc[myindex]= targetPc;
            if (pred_dst != targetPc )
                mystats.flush_num++;
            return;
        }
        updatetabledown(indexingt,myindex);
        targetpc[myindex]= targetPc;
        if (pred_dst != pc+4)
            mystats.flush_num++;
        return;
    }

    void BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = (2*histpower)+(btbSize*(1+tagSize+30+historySize));
    }
};

class BTB_GT_GH : public BTB {
public:
    std::vector<unsigned> globaltable;
    int shared;
    int history;
    BTB_GT_GH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int isShare) :
    BTB(btbSize, historySize, tagSize, fsmState),globaltable(histpower,fsmState), shared(isShare), history(0) {}
    ~BTB_GT_GH() = default;

    uint32_t  sharetype(uint32_t pc1, int shared){
        if (shared==0){
            return history;
        }
        if (shared==1){
            return ((history ^ (pc1/4))%histpower);
        }
        return ((history ^ (pc1/ power2tox(16)))%histpower);
    }

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        uint32_t indexingt = sharetype(pc,shared);
        if (tag[myindex]!=mytag || !valid[myindex]){
            *dst=pc+4;
            return false;
        }
        if (globaltable[indexingt]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[myindex];
        return true;
    }

    void updatetableup(uint32_t indexingt){
        if (globaltable[indexingt]<3)
            globaltable[indexingt]++;
        history=((history*2+1)%histpower);
    }
    void updatetabledown(uint32_t indexingt){
        if (globaltable[indexingt]>0)
            globaltable[indexingt]--;
        history=((history*2)%histpower);
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        uint32_t indexingt = sharetype(pc,shared);
        if (!valid[myindex] || tag[myindex]!=mytag){
            valid[myindex]= true;
            tag[myindex]=mytag;
            targetpc[myindex]= targetPc;
            if (!taken)
                updatetabledown(indexingt);
            else{
                mystats.flush_num++;
                updatetableup(indexingt);
            }
            return;
        }
        if (taken){
            updatetableup(indexingt);
            targetpc[myindex]= targetPc;
            if (pred_dst != targetPc)
                mystats.flush_num++;
            return;
        }
        updatetabledown(indexingt);
        targetpc[myindex]= targetPc;
        if (pred_dst != pc+4)
            mystats.flush_num++;
        return;
    }

    void  BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = (btbSize*(1+tagSize+30))+(2*histpower+historySize);
    }
};

class BTB_LT_GH : public BTB {
public:
    std::vector<std::vector<unsigned >> localtables;
    int history;
    BTB_LT_GH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) :
    BTB(btbSize, historySize, tagSize, fsmState), localtables(btbSize, std::vector<unsigned>(histpower, fsmState)),history(0) {}
    ~BTB_LT_GH() = default;

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        if (tag[myindex]!=mytag || !valid[myindex]){
            *dst=pc+4;
            return false;
        }
        if (localtables[myindex][history]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[myindex];
        return true;
    }

    void updatetableup(uint32_t myindex) {
        if (localtables[myindex][history]<3)
            localtables[myindex][history]++;
        history=((history*2+1)% histpower);
    }
    void updatetabledown(uint32_t myindex){
        if (localtables[myindex][history]>0)
            localtables[myindex][history]--;
        history=((history*2)% histpower);
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        if (!valid[myindex] || tag[myindex]!=mytag){
            for (int i = 0; i < histpower; ++i) {
                localtables[myindex][i]=fsmState;
            }
            valid[myindex]= true;
            tag[myindex]=mytag;
            targetpc[myindex]= targetPc;
            if (!taken)
                updatetabledown(myindex);
            else{
                mystats.flush_num++;
                updatetableup(myindex);
            }
            return;
        }
        if (taken){
            updatetableup(myindex);
            targetpc[myindex]= targetPc;
            if (pred_dst != targetPc )
                mystats.flush_num++;
            return;
        }
        updatetabledown(myindex);
        targetpc[myindex]= targetPc;
        if (pred_dst != pc+4)
            mystats.flush_num++;
        return;
    }

    void  BP_GetStats(SIM_stats *curStats) override{
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = historySize+(btbSize*(1+tagSize+30+2*histpower));
    }
};

class BTB_LT_LH : public BTB {
public:
    std::vector<std::vector<unsigned >> localtables;
    std::vector<int> historyvec;
    BTB_LT_LH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) :
    BTB(btbSize, historySize, tagSize, fsmState), localtables(btbSize, std::vector<unsigned>(histpower, fsmState)),historyvec(btbSize,0) {}
    ~BTB_LT_LH() = default;

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        if (tag[myindex]!=mytag || !valid[myindex]){
            *dst=pc+4;
            return false;
        }
        if (localtables[myindex][historyvec[myindex]]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[myindex];
        return true;
    }

    void updatetableup(uint32_t myindex){
        if (localtables[myindex][historyvec[myindex]]<3)
            localtables[myindex][historyvec[myindex]]++;
        historyvec[myindex]=(historyvec[myindex]*2+1) % histpower;
    }
    void updatetabledown(uint32_t myindex){
        if (localtables[myindex][historyvec[myindex]]>0)
            localtables[myindex][historyvec[myindex]]--;
        historyvec[myindex]=((historyvec[myindex]*2) % histpower);
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t myindex=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        if (!valid[myindex] || tag[myindex]!=mytag){
            historyvec[myindex]=0;
            for (int i = 0; i < histpower; ++i) {
                localtables[myindex][i]=fsmState;
            }
            valid[myindex]= true;
            tag[myindex]=mytag;
            targetpc[myindex]= targetPc;
            if (!taken)
                updatetabledown(myindex);
            else {
                mystats.flush_num++;
                updatetableup(myindex);
            }
            return;
        }
        if (taken){
            updatetableup(myindex);
            targetpc[myindex]= targetPc;
            if (pred_dst != targetPc )
                mystats.flush_num++;
            return;
        }
        updatetabledown(myindex);
        targetpc[myindex]= targetPc;
        if (pred_dst != pc+4)
            mystats.flush_num++;
        return;
    }

    void BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = btbSize*(1+tagSize+30+2*histpower+historySize);
    }
};

BTB *mybtb;
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
    if(isGlobalHist && isGlobalTable){
        mybtb = new BTB_GT_GH(btbSize,historySize,tagSize,fsmState, Shared);
    }
    else if(isGlobalHist && !isGlobalTable){
        mybtb = new BTB_LT_GH(btbSize,historySize,tagSize,fsmState);
    }
    else if(!isGlobalHist && isGlobalTable){
        mybtb = new BTB_GT_LH(btbSize,historySize,tagSize,fsmState, Shared);
    }
    else if(!isGlobalHist && !isGlobalTable){
        mybtb = new BTB_LT_LH(btbSize,historySize,tagSize,fsmState);
    }
    if(mybtb){
        return 0;
    }
	return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
    return mybtb->predict(pc,dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    return mybtb->update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
    return mybtb->BP_GetStats(curStats);
}
