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
    std::vector<uint32_t> targetpc;
    std::vector<uint32_t> tag;
    std::vector<bool> valid;
    int bits_for_btb;
    int bitstotag;
    SIM_stats mystats = {0, 0, 0};
    int bitstotagpower;
    int tagpower;
    int histpower;

    BTB(unsigned btbsize, unsigned historysize, unsigned tagsize, unsigned fsmstate):btbSize(btbsize),
        historySize(historysize), tagSize(tagsize), fsmState(fsmstate),targetpc(btbsize, 0),
        tag(btbsize, 0), valid(btbsize, false){
        bits_for_btb = log2tox(btbsize);
        bitstotag = 2+bits_for_btb;
        bitstotagpower = power2tox(bitstotag);
        tagpower = power2tox(tagSize);
        histpower = power2tox(historySize);
    }
    virtual ~BTB() = 0;
    virtual bool  predict(uint32_t pc, uint32_t *dst) =0;
    virtual void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) = 0;
    virtual void  BP_GetStats(SIM_stats *curStats) = 0;
};

class BTB_GT_GH : public BTB {
public:
    std::vector<unsigned> globaltable;
    int shared;
    int history;

    BTB_GT_GH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int isShare) :
    BTB(btbSize, historySize, tagSize, fsmState),globaltable(histpower,fsmState), shared(isShare), history(0) {}

    ~BTB_GT_GH() override{}

    uint32_t  sharetype(uint32_t pc1, int shared){
        if (shared==0){
            return history;
        }
        if (shared==1){
            return ((history ^ (pc1/4))%histpower);
        }
        if (shared==2){
            return ((history ^ (pc1/ power2tox(16)))%histpower);
        }
        return 0;
    }

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        uint32_t indexingt = sharetype(pc,shared);
        if (tag[indexintag]!=mytag || !valid[indexintag]){
            *dst=pc+4;
            return false;
        }
        if (globaltable[indexingt]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[indexintag];
        return true;
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        uint32_t indexingt = sharetype(pc,shared);
        if (!valid[indexintag] || tag[indexintag]!=mytag){
            valid[indexintag]= true;
            tag[indexintag]=mytag;
            targetpc[indexintag]= targetPc;
            if (taken){
                mystats.flush_num++;
                if (globaltable[indexingt]<3){
                    globaltable[indexingt]++;
                }
                history=(history*2+1) % histpower;
            }
            else{
                if (globaltable[indexingt]>0){
                    globaltable[indexingt]--;
                }
                history=(history*2) % histpower;
            }
            return;
        }
        if (!taken){
            if (pred_dst != pc+4)
                mystats.flush_num++;
            if (globaltable[indexingt]>0)
                globaltable[indexingt]--;
            history=(history*2)% histpower);
            targetpc[indexintag]= targetPc;
            return;
        }
        if (pred_dst != targetPc )
            mystats.flush_num++;
        if (globaltable[indexingt]<3)
            globaltable[indexingt]++;
        history=(history*2+1)% histpower);
        targetpc[indexintag]= targetPc;
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

    ~BTB_LT_GH() override{}

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        if (tag[indexintag]!=mytag || !valid[indexintag]){
            *dst=pc+4;
            return false;
        }
        if (localtables[indexingt][history]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[indexintag];
        return true;
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        if (!valid[indexintag] || tag[indexintag]!=mytag){
            for (int i = 0; i < histpower; ++i) {
                localtables[indexintag][i]=fsmState;
            }
            valid[indexintag]= true;
            tag[indexintag]=mytag;
            targetpc[indexintag]= targetPc;
            if (taken){
                mystats.flush_num++;
                if (localtables[indexingt][history]<3){
                    localtables[indexingt][history]++;
                }
                history=(history*2+1) % histpower;
            }
            else{
                if (localtables[indexingt][history]>0){
                    localtables[indexingt][history]--;
                }
                history=(history*2) % histpower;
            }
            return;
        }
        if (!taken){
            if (pred_dst != pc+4)
                mystats.flush_num++;
            if (localtables[indexingt][history]>0)
                localtables[indexingt][history]--;
            history=(history*2)% histpower);
            targetpc[indexintag]= targetPc;
            return;
        }
        if (pred_dst != targetPc )
            mystats.flush_num++;
        if (localtables[indexingt][history]<3)
            localtables[indexingt][history]++;
        history=(history*2+1)% histpower);
        targetpc[indexintag]= targetPc;
    }

    void  BP_GetStats(SIM_stats *curStats) override{
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = historySize+(btbSize*(1+tagSize+30+2*histpower));
    }
};

class BTB_GT_LH : public BTB {
public:
    std::vector<unsigned> globaltable;
    int shared;
    std::vector<int> historyvec;
    BTB_GT_LH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,int isShare) :
    BTB(btbSize, historySize, tagSize, fsmState),globaltable(histpower,fsmState),shared(isShare),historyvec(btbSize,0){}

    ~BTB_GT_LH() override{}

    uint32_t  sharetype1(uint32_t pc1, int shared,uint32_t indexintag){
        if (shared==0){
            return historyvec[indexintag];
        }
        if (shared==1){
            return ((historyvec[indexintag] ^ (pc1/4))%histpower);
        }
        if (shared==2){
            return ((historyvec[indexintag] ^ (pc1/ power2tox(16)))%histpower);
        }
        return 0;
    }

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        uint32_t indexingt = sharetype1(pc,shared,indexintag);
        if (tag[indexintag]!=mytag || !valid[indexintag]){
            *dst=pc+4;
            return false;
        }
        if (globaltable[indexingt]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[indexintag];
        return true;
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        uint32_t indexingt = sharetype1(pc,shared,indexintag);
        if (!valid[indexintag] || tag[indexintag] != tmp_tag) {
            historyvec[indexintag] = 0;
        }
        if (!valid[indexintag] || tag[indexintag]!=mytag){
            valid[indexintag]= true;
            tag[indexintag]=mytag;
            targetpc[indexintag]= targetPc;
            if (taken){
                mystats.flush_num++;
                if (globaltable[indexingt]<3){
                    globaltable[indexingt]++;
                }
                historyvec[indexintag]=(historyvec[indexintag]*2+1) % histpower;
            }
            else{
                if (globaltable[indexingt]>0){
                    globaltable[indexingt]--;
                }
                historyvec[indexintag]=(historyvec[indexintag]*2) % histpower;
            }
            return;
        }
        if (!taken){
            if (pred_dst != pc+4)
                mystats.flush_num++;
            if (globaltable[indexingt]>0)
                globaltable[indexingt]--;
            historyvec[indexintag]=(historyvec[indexintag]*2)% histpower);
            targetpc[indexintag]= targetPc;
            return;
        }
        if (pred_dst != targetPc )
            mystats.flush_num++;
        if (globaltable[indexingt]<3)
            globaltable[indexingt]++;
        historyvec[indexintag]=(historyvec[indexintag]*2+1)% histpower);
        targetpc[indexintag]= targetPc;
    }

    void BP_GetStats(SIM_stats *curStats) override {
        curStats->flush_num = mystats.flush_num;
        curStats->br_num = mystats.br_num;
        curStats->size = (2*histpower)+(btbSize*(1+tagSize+30+historySize));
    }
};

class BTB_LT_LH : public BTB {
public:
    std::vector<std::vector<unsigned >> localtables;
    std::vector<int> historyvec;

    BTB_LT_LH(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState) :
    BTB(btbSize, historySize, tagSize, fsmState), localtables(btbSize, std::vector<unsigned>(histpower, fsmState)),historyvec(btbSize,0) {}

    ~BTB_LT_LH() override{}

    bool  predict(uint32_t pc, uint32_t *dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        if (tag[indexintag]!=mytag || !valid[indexintag]){
            *dst=pc+4;
            return false;
        }
        if (localtables[indexintag][historyvec[indexintag]]<2){
            *dst=pc+4;
            return false;
        }
        *dst=targetpc[indexintag];
        return true;
    }

    void  update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) override {
        uint32_t indexintag=(pc/4)%btbSize;
        uint32_t mytag=((pc/bitstotagpower)%tagpower);
        mystats.br_num++;
        if (!valid[indexintag] || tag[indexintag]!=mytag){
            historyvec[indexintag]=0;
            for (int i = 0; i < histpower; ++i) {
                localtables[indexintag][i]=fsmState;
            }
            valid[indexintag]= true;
            tag[indexintag]=mytag;
            targetpc[indexintag]= targetPc;
            if (taken){
                mystats.flush_num++;
                if (localtables[indexintag][historyvec[indexintag]]<3){
                    localtables[indexintag][historyvec[indexintag]]++;
                }
                historyvec[indexintag]=(historyvec[indexintag]*2+1) % histpower;
            }
            else{
                if (localtables[indexintag][historyvec[indexintag]]>0){
                    localtables[indexintag][historyvec[indexintag]]--;
                }
                historyvec[indexintag]=(historyvec[indexintag]*2) % histpower;
            }
            return;
        }
        if (!taken){
            if (pred_dst != pc+4)
                mystats.flush_num++;
            if (localtables[indexintag][historyvec[indexintag]]>0)
                localtables[indexintag][historyvec[indexintag]]--;
            historyvec[indexintag]=(historyvec[indexintag]*2)% histpower);
            targetpc[indexintag]= targetPc;
            return;
        }
        if (pred_dst != targetPc )
            mystats.flush_num++;
        if (localtables[indexintag][historyvec[indexintag]]<3)
            localtables[indexintag][historyvec[indexintag]]++;
        historyvec[indexintag]=(historyvec[indexintag]*2+1)% histpower);
        targetpc[indexintag]= targetPc;
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
    else if(!isGlobalHist && isGlobalTable){
        mybtb = new BTB_GT_LH(btbSize,historySize,tagSize,fsmState, Shared);
    }
    else if(isGlobalHist && !isGlobalTable){
        mybtb = new BTB_LT_GH(btbSize,historySize,tagSize,fsmState);
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
    mybtb->BP_GetStats(curStats);
    delete mybtb;
    return;
}

