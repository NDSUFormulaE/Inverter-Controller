class GPIOHandler
{
    public:
        bool Init(void);
        uint16_t GetPedalSpeed();
        uint16_t GetClearPin();
        uint16_t GetPedalTorque();
        void UpdateDisplays();
        void LcdUpdateState(char* str_to_print);
        void LcdUpdateError1(char* str_to_print);
        void LcdUpdateError2(char* str_to_print);
        void LcdUpdateError3(char* str_to_print);
    private:
        uint16_t speed;
        bool LcdInit();
        void LCDDisplaySAE();
        
};