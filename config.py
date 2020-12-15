import numpy as np


def init_config(max_num_sub_frames=4):
    config = {
        "channelCfg": {},
        "dataPath": [],
        "profileCfg": [],
        "frameCfg": {},
        "guiMonitor": [],
        "extendedMaxVelocity": [],
        "dfeDataOutputMode": {},
        "advFrameCfg": {},
        "subFrameCfg": [],
        "chirpCfg": [],
        "subFrameInfo": [],
        "log2linScale": [],
        "platform": "xWR18xx",
        "cmdReceivedFlag": {},
        "numDetectedObj": [],
        "dspFftScaleComp2D_lin": [],
        "dspFftScaleComp2D_log": [],
        "dspFftScaleComp1D_lin": [],
        "dspFftScaleComp1D_log": [],
        "dspFftScaleCompAll_lin": [],
        "dspFftScaleCompAll_log": [],
        "interFrameProcessingTime": [],
        "transmitOutputTime": [],
        "interFrameProcessingMargin": [],
        "interChirpProcessingMargin": [],
        "activeFrameCPULoad": [],
        "interFrameCPULoad": [],
        "compRxChanCfg": {},
        "measureRxChanCfg": {},
        "bpmCfg": [],
        "nearFieldCfg": [],
        "aoaFovCfg": [],
        "cfarRangeFov": [],
        "cfarDopplerFov": []
    }

    # initialize variables
    for i in range(max_num_sub_frames):
        config["dataPath"].append({
            "numTxAzimAnt": 0,
            "numTxElevAnt": 0,
            "numRxAnt": 0,
            "azimuthResolution": 0,
            "numChirpsPerFrame": 0,
            "numDopplerBins": 0,
            "numRangeBins": 0,
            "rangeResolutionMeters": 0,
            "rangeMeters": 0,
            "velocityMps": 0,
            "dopplerResolutionMps": 0
        })

        config["log2linScale"].append(0)

        config["extendedMaxVelocity"].append({"enable": 0})

        config["guiMonitor"].append({
            "subFrameIdx": 0,
            "detectedObjects": 0,
            "logMagRange": 0,
            "noiseProfile": 0,
            "rangeAzimuthHeatMap": 0,
            "rangeDopplerHeatMap": 0,
            "statsInfo": 0
        })

    config["dfeDataOutputMode"]["mode"] = 0
    return config


def check_sub_frame_idx(config, sub_frame_num, command):
    if config["dfeDataOutputMode"]["mode"] == 1:
        if sub_frame_num != -1:
            print("%s SubFrameIdx must be set to -1 (i.e. N/A)." % command)
            return -1
        return 0
    elif config["dfeDataOutputMode"]["mode"] == 3:
        if (sub_frame_num >= 4) or (sub_frame_num < -1):
            print("%s SubFrameIdx is invalid." % command);
            return -1
        return 0
    else:
        print("Make sure dfeDataOutputMode has been configured before %s" % command)


def validate_config(path):
    profile = open(path, "r")
    lines = profile.readlines()
    config = init_config()
    profileCfgCounter = 0
    chirpCfgCounter = 0
    for line in lines:
        tokens = line.rstrip().split()
        if tokens[0] == "channelCfg":
            config["channelCfg"]["txChannelEn"] = int(tokens[2])
            config["channelCfg"]["numTxAzimAnt"] = ((config["channelCfg"]["txChannelEn"] << 0) & 1) + \
                                                   ((config["channelCfg"]["txChannelEn"] >> 2) & 1)
            config["channelCfg"]["numTxElevAnt"] = ((config["channelCfg"]["txChannelEn"] >> 1) & 1)
            config["channelCfg"]["rxChannelEn"] = int(tokens[1])
            config["channelCfg"]["numRxAnt"] = ((config["channelCfg"]["rxChannelEn"] << 0) & 1) + \
                                               ((config["channelCfg"]["rxChannelEn"] >> 1) & 1) + \
                                               ((config["channelCfg"]["rxChannelEn"] >> 2) & 1) + \
                                               ((config["channelCfg"]["rxChannelEn"] >> 3) & 1)
        elif tokens[0] == "profileCfg":
            config["profileCfg"].append({
                "profileId": int(tokens[1]),
                "startFreq": float(tokens[2]),
                "idleTime": float(tokens[3]),
                "adcStartTimeConst": float(tokens[4]),
                "rampEndTime": float(tokens[5]),
                "freqSlopeConst": float(tokens[8]),
                "numAdcSamples": int(tokens[10]),
                "digOutSampleRate": float(tokens[11])
            })
            profileCfgCounter += 1
        elif tokens[0] == "chirpCfg":
            config["chirpCfg"].append({
                "startIdx": int(tokens[1]),
                "endIdx": int(tokens[2]),
                "profileId": int(tokens[3]),
                "txEnable": int(tokens[8]),
                "numTxAzimAnt": 0
            })
            if config["chirpCfg"][chirpCfgCounter]["txEnable"] == 5:
                config["chirpCfg"][chirpCfgCounter]["numTxAzimAnt"] = 1
            chirpCfgCounter += 1
        elif tokens[0] == "frameCfg":
            if config["dfeDataOutputMode"]["mode"] != 1:
                print("frameCfg can only be used with dfeDataOutputMode 1")
                return None
            config["frameCfg"]["chirpStartIdx"] = int(tokens[1])
            config["frameCfg"]["chirpEndIdx"] = int(tokens[2])
            config["frameCfg"]["numLoops"] = int(tokens[3])
            config["frameCfg"]["numFrames"] = int(tokens[4])
            config["frameCfg"]["framePeriodicity"] = float(tokens[5])
        elif tokens[0] == "extendedMaxVelocity":
            subFrameMaxVel = int(tokens[1])
            if check_sub_frame_idx(config, subFrameMaxVel, "extendedMaxVelocity") == -1:
                return None
            if len(tokens) != 3:
                print("extendedMaxVelocity invalid number of arguments")
                return None
            if subFrameMaxVel == -1:
                for maxVelIdx in range(4):
                    config["extendedMaxVelocity"][maxVelIdx]["enable"] = int(tokens[2])
            else:
                config["extendedMaxVelocity"][subFrameMaxVel]["enable"] = int(tokens[2])
        elif tokens[0] == "guiMonitor":
            if len(tokens) != 8:
                print("guiMonitor invalid number of arguments")
                return None
            guiMonIdx = int(tokens[1])
            if check_sub_frame_idx(config, guiMonIdx, "guiMonitor") == -1:
                return None
            if guiMonIdx == -1:
                for guiIdx in range(4):
                    config["guiMonitor"][guiIdx] = {
                        "subFrameIdx": int(tokens[1]),
                        "detectedObjects": int(tokens[2]),
                        "logMagRange": int(tokens[3]),
                        "noiseProfile": int(tokens[4]),
                        "rangeAzimuthHeatMap": int(tokens[5]),
                        "rangeDopplerHeatMap": int(tokens[6]),
                        "statsInfo": int(tokens[7])
                    }
            else:
                config["guiMonitor"][guiMonIdx] = {
                    "subFrameIdx": int(tokens[1]),
                    "detectedObjects": int(tokens[2]),
                    "logMagRange": int(tokens[3]),
                    "noiseProfile": int(tokens[4]),
                    "rangeAzimuthHeatMap": int(tokens[5]),
                    "rangeDopplerHeatMap": int(tokens[6]),
                    "statsInfo": int(tokens[7])
                }
        elif tokens[0] == "dfeDataOutputMode":
            if len(tokens) != 2:
                print("dfeDataOutputMode invalid number of arguments")
                return None
            config["dfeDataOutputMode"]["mode"] = int(tokens[1])
        elif tokens[0] == "advFrameCfg":
            if len(tokens) != 6:
                print("advFrameCfg invalid number of arguments")
                return None
            if config["dfeDataOutputMode"]["mode"] != 3:
                print("advFrameCfg must use dfeDataOutputMode 3")
                return None
            config["advFrameCfg"]["numOfSubFrames"] = int(tokens[1])
            config["advFrameCfg"]["forceProfile"] = int(tokens[2])
            config["advFrameCfg"]["numFrames"] = int(tokens[3])
            config["advFrameCfg"]["triggerSelect"] = int(tokens[4])
            config["advFrameCfg"]["frameTrigDelay"] = int(tokens[5])
            if config["advFrameCfg"]["numOfSubFrames"] > 4:
                print("advFrameCfg: Maximum number of subframes is 4")
                return None
        elif tokens[0] == "subFrameCfg":
            if len(tokens) != 11:
                print("subFrameCfg invalid number of arguments")
                return None
            if config["dfeDataOutputMode"]["mode"] != 3:
                print("subFrameCfg is allowed only in advFrameCfg mode and must use dfeDataOutputMode 3")
                return None
            subFrameNumLocal = int(tokens[1])
            if subFrameNumLocal >= 4:
                print("Bad subframe config:Invalid subframe number")
                return None
            config["subFrameCfg"].append({
                "forceProfileIdx": int(tokens[2]),
                "chirpStartIdx": int(tokens[3]),
                "numOfChirps": int(tokens[4]),
                "numLoops": int(tokens[5]),
                "burstPeriodicity": float(tokens[6]),
                "chirpStartIdxOffset": int(tokens[7]),
                "numOfBurst": int(tokens[8]),
                "numOfBurstLoops": int(tokens[9]),
                "subFramePeriodicity": float(tokens[10])
            })
            if config["subFrameCfg"][subFrameNumLocal]["numOfBurst"] != 1:
                print("Bad subframe config: numOfBurst must be 1")
                return None
            if config["subFrameCfg"][subFrameNumLocal]["numOfBurstLoops"] != 1:
                print("Bad subframe config: numOfBurstLoops must be 1")
                return None
        elif tokens[0] == "cfarCfg":
            if len(tokens) != 10:
                print(" invalid number of arguments")
                return None
            localSubframe = int(tokens[1])
            threshold = float(tokens[8])
            peakGroupingEn = int(tokens[9])
            if peakGroupingEn != 0 and peakGroupingEn != 1:
                print("cfarCfg invalid peakGroupingEn value.")
                return None
            if check_sub_frame_idx(config, localSubframe, "cfarCfg") == -1:
                return None
        elif tokens[0] == "compRangeBiasAndRxChanPhase":
            # 3*4*2+1+1
            if len(tokens) != 26:
                print("compRangeBiasAndRxChanPhase invalid number of arguments")
                return None
            config["compRxChanCfg"]["rangeBias"] = float(tokens[1])
        elif tokens[0] == "measureRangeBiasAndRxChanPhase":
            if len(tokens) != 4:
                print("measureRangeBiasAndRxChanPhase invalid number of arguments")
                return None
            config["measureRxChanCfg"]["enabled"] = int(tokens[1])  # 0 - compensation, 1 - measurement
        elif tokens[0] == "CQRxSatMonitor":
            if len(tokens) != 6:
                print("CQRxSatMonitor invalid number of arguments")
                return None
        elif tokens[0] == "CQSigImgMonitor":
            if len(tokens) != 4:
                print("CQSigImgMonitor invalid number of arguments")
                return None
        elif tokens[0] == "analogMonitor":
            if len(tokens) != 3:
                print("analogMonitor invalid number of arguments")
                return None
        elif tokens[0] == "multiObjBeamForming":
            localSubframe = int(tokens[1])
            if len(tokens) != 4:
                print("multiObjBeamForming invalid number of arguments")
                return None
            if check_sub_frame_idx(config, localSubframe, "multiObjBeamForming") == -1:
                return None
        elif tokens[0] == "calibDcRangeSig":
            localSubframe = int(tokens[1])
            if len(tokens) != 6:
                print("calibDcRangeSig invalid number of arguments")
                return None
            if check_sub_frame_idx(config, localSubframe, "multiObjBeamForming") == -1:
                return None
        elif tokens[0] == "adcbufCfg":
            localSubframe = int(tokens[1])
            if len(tokens) != 6:
                print("adcbufCfg invalid number of arguments")
                return None
            if check_sub_frame_idx(config, localSubframe, "multiObjBeamForming") == -1:
                return None
        elif tokens[0] == "adcCfg":
            if len(tokens) != 3:
                print("adcCfg invalid number of arguments")
                return None
        elif tokens[0] == "clutterRemoval":
            if len(tokens) != 3:
                print("clutterRemoval invalid number of arguments")
                return None
        elif tokens[0] == "bpmCfg":
            print("bpmCfg command not supported")
            return None
        elif tokens[0] == "lvdsStreamCfg":
            if len(tokens) != 5:
                print("lvdsStreamCfg invalid number of arguments")
                return None
            lvdsStreamingSubframeIdx = int(tokens[1])
            if check_sub_frame_idx(config, lvdsStreamingSubframeIdx, "lvdsStreamCfg") == -1:
                return None
        elif tokens[0] == "nearFieldCfg":
            print("bpmCfg command not supported")
            return None
        elif tokens[0] == "lowPower":
            if len(tokens) != 3:
                print("lowPower invalid number of arguments")
                return None
        elif tokens[0] == "cfarFovCfg":
            if len(tokens) != 5:
                print("cfarFovCfg invalid number of arguments")
                return None
            cfarFovSubframeIdx = int(tokens[1])
            cfarFovDir = int(tokens[2])
            cfarFovMin = float(tokens[3])
            cfarFovMax = float(tokens[4])
            if check_sub_frame_idx(config, cfarFovSubframeIdx, "cfarFovCfg") == -1:
                return None
            if cfarFovMin > cfarFovMax:
                print("cfarFovCfg invalid min/max configuration: Min > Max.")
                return None
            if cfarFovDir == 0:
                if cfarFovMin < 0:
                    print("cfarFovCfg invalid min/max configuration: Min <0.")
                    return None
                if cfarFovSubframeIdx == -1:
                    for cfarFovIdx in range(4):
                        config["cfarRangeFov"].append({
                            "min": cfarFovMin,
                            "max": cfarFovMax
                        })
                else:
                    config["cfarRangeFov"].append({
                        "min": cfarFovMin,
                        "max": cfarFovMax
                    })

            elif cfarFovDir == 1:
                if cfarFovSubframeIdx == -1:
                    for cfarFovIdx in range(4):
                        config["cfarDopplerFov"].append({
                            "min": cfarFovMin,
                            "max": cfarFovMax
                        })
                else:
                    config["cfarDopplerFov"].append({
                        "min": cfarFovMin,
                        "max": cfarFovMax
                    })
            else:
                print("cfarFovCfg invalid procDirection")
                return None
        elif tokens[0] == "aoaFovCfg":
            if len(tokens) != 6:
                print("aoaFovCfg invalid number of arguments")
                return None
            aoaFovSubframeIdx = int(tokens[1])
            aoaFovMinAzim = float(tokens[2])
            aoaFovMaxAzim = float(tokens[3])
            aoaFovMinElev = float(tokens[4])
            aoaFovMaxElev = float(tokens[5])

            if aoaFovMinAzim > aoaFovMaxAzim:
                print("aoaFovCfg invalid azimuth min/max configuration: Min > Max.")
                return None
            if aoaFovMinElev > aoaFovMaxElev:
                print("aoaFovCfg invalid elevation min/max configuration: Min > Max.")
                return None
            if aoaFovMinAzim < -90:
                print("aoaFovCfg invalid min azimuth angle. Minimum allowed value is -90.")
                return None
            if aoaFovMaxAzim > 90:
                print("aoaFovCfg invalid max azimuth angle. Maximum allowed value is 90.")
                return None
            if aoaFovMinElev < -90:
                print("aoaFovCfg invalid min elevation angle. Minimum allowed value is -90.")
                return None
            if aoaFovMaxElev > 90:
                print("aoaFovCfg invalid max elevation angle. Maximum allowed value is 90.")
                return None
            if check_sub_frame_idx(config, aoaFovSubframeIdx, "aoaFovCfg") == -1:
                return None
            if aoaFovSubframeIdx == -1:
                for aoaFovIdx in range(4):
                    config["aoaFovCfg"].append({
                        "minAzim": aoaFovMinAzim,
                        "maxAzim": aoaFovMaxAzim,
                        "minElev": aoaFovMinElev,
                        "manElev": aoaFovMaxElev
                    })
            else:
                config["aoaFovCfg"].append({
                    "minAzim": aoaFovMinAzim,
                    "maxAzim": aoaFovMaxAzim,
                    "minElev": aoaFovMinElev,
                    "manElev": aoaFovMaxElev
                })
        elif tokens[0] == "calibData":
            if int(tokens[1]) + int(tokens[2]) > 1:
                print("calibData invalid configuration")
                return None

    return profileCfgCounter, chirpCfgCounter, config


def get_profile_idx(config, sub_frame_num, profile_cfg_counter, chirp_cfg_counter):
    firstChirp = 0
    if config["dfeDataOutputMode"]["mode"] == 1:
        firstChirp = config["frameCfg"]["chirpStartIdx"]
    elif config["dfeDataOutputMode"]["mode"] == 3:
        firstChirp = config["subFrameCfg"][sub_frame_num]["chirpStartIdx"]

    profileId = -1
    for i in range(chirp_cfg_counter):
        if (firstChirp >= config["chirpCfg"][i]["startIdx"]) and (firstChirp <= config["chirpCfg"][i]["endIdx"]):
            profileId = config["chirpCfg"][i]["profileId"]
    if profileId == -1:
        return -1
    for i in range(profile_cfg_counter):
        if config["profileCfg"][i]["profileId"] == profileId:
            return i
    return -1


def get_ant_config(ParamsIn, subFrameNum):
    if ParamsIn["dfeDataOutputMode"]["mode"] == 1:
        if (ParamsIn["chirpCfg"][0]["numTxAzimAnt"] == 1):
            ParamsIn["dataPath"][0]["dataPath"] = 1
        else:
            ParamsIn["dataPath"][0]["numTxAzimAnt"] = ParamsIn["channelCfg"]["numTxAzimAnt"]
        ParamsIn["dataPath"][0]["numTxElevAnt"] = ParamsIn.channelCfg.numTxElevAnt
        ParamsIn["dataPath"][0]["numTxAzimAnt"] = ParamsIn.channelCfg.numRxAnt


def parse_config(path):
    profile_cfg_counter, chirp_cfg_counter, config = validate_config(path)
    if config is None:
        print("Parsing configuration failed.")
        return None
    totalSubframes = 0
    if config["dfeDataOutputMode"]["mode"] == 1:
        totalSubframes = 1
    elif config["dfeDataOutputMode"]["mode"] == 3:
        totalSubframes = config["advFrameCfg"]["numOfSubFrames"]

    for idx in range(totalSubframes):
        profileCfgIdx = get_profile_idx(config, idx, profile_cfg_counter, chirp_cfg_counter)
        if profileCfgIdx == -1:
            print("Could not find profile for chirp configuration")
            return None

        if config["dfeDataOutputMode"]["mode"] == 1:
            if config["chirpCfg"][0]["numTxAzimAnt"] == 1:
                config["dataPath"][0]["dataPath"] = 1
            else:
                config["dataPath"][0]["numTxAzimAnt"] = config["channelCfg"]["numTxAzimAnt"]
            config["dataPath"][0]["numTxElevAnt"] = config["channelCfg"]["numTxElevAnt"]
            config["dataPath"][0]["numRxAnt"] = config["channelCfg"]["numRxAnt"]

            config["dataPath"][idx]["numChirpsPerFrame"] = (config["frameCfg"]["chirpEndIdx"] - \
                                                            config["frameCfg"]["chirpStartIdx"] + 1) * \
                                                           config["frameCfg"]["numLoops"]

        elif config["dfeDataOutputMode"]["mode"] == 3:
            pass

        config["dataPath"][idx]["numTxAnt"] = config["dataPath"][idx]["numTxElevAnt"] + \
                                              config["dataPath"][idx]["numTxAzimAnt"]
        config["dataPath"][idx]["numDopplerChirps"] = config["dataPath"][idx]["numChirpsPerFrame"] / \
                                                      config["dataPath"][idx]["numTxAnt"]

        if config["dataPath"][idx]["numRxAnt"] * config["dataPath"][idx]["numTxAzimAnt"] < 2:
            config["dataPath"][idx]["azimuthResolution"] = 'None'
        else:
            config["dataPath"][idx]["azimuthResolution"] = np.round(np.arcsin(
                2 / (config["dataPath"][idx]["numRxAnt"] * config["dataPath"][idx]["numTxAzimAnt"])) * 180 / np.pi, 1)

        if config["dataPath"][idx]["numDopplerChirps"] <= 4:
            config["dataPath"][idx]["numDopplerBins"] = 8
        else:
            config["dataPath"][idx]["numDopplerBins"] = 1 << np.ceil(
                np.log2(config["dataPath"][idx]["numDopplerChirps"])).astype(np.int)

        config["dataPath"][idx]["numRangeBins"] = 1 << np.ceil(
            np.log2(config["profileCfg"][profileCfgIdx]["numAdcSamples"])).astype(np.int)

        if ((config["dataPath"][idx]["numTxAnt"] * config["dataPath"][idx]["numRxAnt"]) == 12) and (
                config["dataPath"][idx]["numRangeBins"] == 1024):
            config["dataPath"][idx]["numRangeBins"] = 1022

        CLI_FREQ_SCALE_FACTOR = 3.6  # 77GHz
        if config["profileCfg"][profileCfgIdx]["startFreq"] < 76:
            CLI_FREQ_SCALE_FACTOR = 2.7  # 60GHz

        mmwFreqSlopeConst = np.trunc(
            config["profileCfg"][profileCfgIdx]["freqSlopeConst"] * (1 << 26) / CLI_FREQ_SCALE_FACTOR)
        config["profileCfg"][profileCfgIdx]["freqSlopeConst_actual"] = mmwFreqSlopeConst * (
                CLI_FREQ_SCALE_FACTOR / (1 << 26))
        startFreqConst = np.trunc(config["profileCfg"][profileCfgIdx]["startFreq"] * (1 << 26) / CLI_FREQ_SCALE_FACTOR)
        config["profileCfg"][profileCfgIdx]["startFreq_actual"] = (startFreqConst * CLI_FREQ_SCALE_FACTOR / (1 << 26))

        config["profileCfg"][profileCfgIdx]["centerFreq_actual"] = (
                config["profileCfg"][profileCfgIdx]["startFreq_actual"] + \
                0.5 * ((config["profileCfg"][profileCfgIdx]["freqSlopeConst_actual"] * \
                        config["profileCfg"][profileCfgIdx]["numAdcSamples"]) / \
                       (config["profileCfg"][profileCfgIdx]["digOutSampleRate"])) + \
                (config["profileCfg"][profileCfgIdx]["freqSlopeConst_actual"] * \
                 config["profileCfg"][profileCfgIdx]["adcStartTimeConst"] * 10 * 1e-9)
        )

        config["dataPath"][idx]["rangeResolutionMeters"] = 300 * \
                                                           config["profileCfg"][profileCfgIdx]["digOutSampleRate"] / \
                                                           (2 * 1e3 * \
                                                            config["profileCfg"][profileCfgIdx]["numAdcSamples"] * \
                                                            config["profileCfg"][profileCfgIdx][
                                                                "freqSlopeConst_actual"])

        config["dataPath"][idx]["rangeIdxToMeters"] = 3e8 * config["profileCfg"][profileCfgIdx]["digOutSampleRate"] * \
                                                      1e3 / (2 * np.abs(
            config["profileCfg"][profileCfgIdx]["freqSlopeConst_actual"]) * 1e12 * config["dataPath"][idx][
                                                                 "numRangeBins"])

        config["dataPath"][idx]["rangeMeters"] = 300 * 0.8 * config["profileCfg"][profileCfgIdx]["digOutSampleRate"] / (
                2 * config["profileCfg"][profileCfgIdx]["freqSlopeConst_actual"] * 1e3)

        config["dataPath"][idx]["velocityMps"] = 3e8 / (
                4 * config["profileCfg"][profileCfgIdx]["centerFreq_actual"] * 1e9 *
                (config["profileCfg"][profileCfgIdx]["idleTime"] + config["profileCfg"][
                    profileCfgIdx]["rampEndTime"]) *
                1e-6 * config["dataPath"][idx]["numTxAnt"])

        config["dataPath"][idx]["dopplerResolutionMps"] = 3e8 / (
                2 * config["profileCfg"][profileCfgIdx]["centerFreq_actual"] * 1e9 *
                (config["profileCfg"][profileCfgIdx]["idleTime"] + config["profileCfg"][
                    profileCfgIdx]["rampEndTime"]) * 1e-6 *
                config["dataPath"][idx]["numDopplerBins"] * config["dataPath"][idx]["numTxAnt"])

        NumVirtAnt = config["dataPath"][idx]["numTxAnt"] * config["dataPath"][idx]["numRxAnt"]

        config["log2linScale"][idx] = (1 / 256) * (2 ** np.ceil(np.log2(NumVirtAnt)) / NumVirtAnt)
        config["toDB"] = 20 * np.log10(2)
        config["rangeAzimuthHeatMapGrid_points"] = 100

        # Range HWA DPU
        config["dspFftScaleComp1D_lin"].append(32 / config["dataPath"][idx]["numRangeBins"])
        config["dspFftScaleComp1D_log"].append(20 * np.log10(config["dspFftScaleComp1D_lin"][idx]))
        # Doppler HWA DPU
        config["dspFftScaleComp2D_lin"].append(1)
        config["dspFftScaleComp2D_log"].append(0)
        config["dspFftScaleCompAll_lin"].append(config["dspFftScaleComp2D_lin"][idx] *\
                                                config["dspFftScaleComp1D_lin"][idx])
        config["dspFftScaleCompAll_log"].append(config["dspFftScaleComp2D_log"][idx] +\
                                                config["dspFftScaleComp1D_log"][idx])
    return config


if __name__ == '__main__':
    config = parse_config("configs/profile_3d.cfg")
    print(config)
