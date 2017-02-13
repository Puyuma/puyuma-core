class HsvThreshold {
    public:
    int h_min;
    int h_max;
    int s_min;
    int s_max;
    int v_min;
    int v_max;

	//constructor using initialization value
    HsvThreshold(int h_min,int h_max, int s_min,int s_max, int v_min, int v_max):
    h_min(h_min),h_max(h_max),s_min(s_min),s_max(s_max),v_min(v_min),v_max(v_max){}

	//constructor using default value 
	HsvThreshold():
	h_min(0),h_max(255),s_min(0),s_max(255),v_min(0),v_max(255){}

    void set_hsv(int h_min,int h_max, int s_min,int s_max, int v_min, int v_max)
    {
        this->h_min=h_min;
        this->h_max=h_max;
        this->s_min=s_min;
        this->s_max=s_max;
        this->v_min=v_min;
        this->v_max=v_max;
    }
};
