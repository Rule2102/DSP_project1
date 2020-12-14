Blue waveform represent MSPWM counter (GPIO toggles on each TBCTR=0/PRD)
Yellow waveform represents execution time (GPIO is set at the begining of dma_isr/multisampling algorithm within dma_isr and clear in the end)

ur8os1_texe_noADC.png: ur8os1 --> texe_dmaisr without ADC = 9.355us; texe_dmaisr with ADC = 10us;
MS_texe.png & MS_texe_zoom.png: --> texe_MScode = 725ns;
ur8os1_texe.png: infinite persistant display (texe < MS_TBPRD)