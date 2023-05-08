import { useState, useCallback, useEffect } from "react";
import { Box, Slider, TextField, Button } from "@mui/material";
import ParameterSlider from "./ParameterSlider";
import FrameComponent from "./FrameComponent";
import PortalPopup from "./PortalPopup";
import styles from "./StatusCard.module.css";
import {Amplify,PubSub} from 'aws-amplify';
import {AWSIoTProvider} from '@aws-amplify/pubsub';

 const StatusCard = () => {
  const [isFramePopupOpen, setFramePopupOpen] = useState(false);
  const [phSetpoint, setPhSetpoint] = useState(7);
  const [conductivitySetpoint, setConductivitySetpoint] = useState(1000);
  const [period, setPeriod] = useState(30);
  const [dutyCycle, setDutyCycle] = useState(50);
  const [currentPh, setCurrentPh] = useState(7);
  const [currentConductivity, setCurrentConductivity] = useState(1000);
  const [currentTemperature, setCurrentTemperature] = useState(25);
  const [currentHumidity, setCurrentHumidity] = useState(85);
  const [currentLightIntensity, setCurrentLightIntensity] = useState(7);
  const [currentOxygenConc, setCurrentOxygenConc] = useState(21);
  const [currentCO2Conc, setCurrentCO2Conc] = useState(400);

  const openFramePopup = useCallback(() => {
    setFramePopupOpen(true);
  }, []);

  const closeFramePopup = useCallback(() => {
    setFramePopupOpen(false);
  }, []);

  const handleUpdateSettingsButtonClick = async () => {
    let command = 1;
    // console.log(phSetpoint);
    // console.log(conductivitySetpoint);

    // Publish data to an AWS IoT topic with multiple parameters
    await PubSub.publish('capstone/sub', {
      // ph: {phSetpoint},
      command,
      phSetpoint: phSetpoint,
      conductivitySetpoint: conductivitySetpoint,
      period: period,
      dutyCycle: dutyCycle
    })
      .then(() => console.log('Message published successfully.'))
      .catch(error => console.error('Error publishing message:', error));
  };

  const handleGetReadingsButtonClick = async () => {
    let command = 2;
    // Publish data to an AWS IoT topic with multiple parameters
    await PubSub.publish('capstone/sub', {
      // ph: {phSetpoint},
      command
    })
      .then(() => console.log('Message published successfully.'))
      .catch(error => console.error('Error publishing message:', error));
  };

  const phCallbackFunction = (childData) => {
    setPhSetpoint(childData)
  };

  const conductivityCallbackFunction = (childData) => {
    setConductivitySetpoint(childData)
  };

  const periodCallbackFunction = (childData) => {
    setPeriod(childData)
  };

  const dutyCycleCallbackFunction = (childData) => {
    setDutyCycle(childData)
  };

  // PubSub.subscribe('myTopic').subscribe({
  //   next: data => console.log('Message received', data),
  //   error: error => console.error(error),
  //   complete: () => console.log('Done'),
  // });
  useEffect(() => {
    // Subscribe to an AWS IoT topic
    const subscription = PubSub.subscribe('capstone/pub').subscribe({
      next: data => {
        setCurrentPh(data.value.ph*1.0/100);
        setCurrentConductivity(data.value.conductivity);
        setCurrentTemperature(data.value.temperature);
        setCurrentHumidity(data.value.humidity);
        setCurrentLightIntensity(data.value.lightIntensity);
        setCurrentOxygenConc(data.value.oxygen*1.0/10);
        setCurrentCO2Conc(data.value.co2);
        console.log('Message received', data);
      },
      error: error => console.error('Error subscribing:', error),
      complete: () => console.log('Subscription completed.')
    });

    return () => {
      // Unsubscribe from the topic when the component is unmounted
      subscription.unsubscribe();
    };
  }, []);


  return (
    <>
      <div className={styles.frameParent}>
        <div className={styles.instanceParent}>
          <ParameterSlider 
            parentCallback = {phCallbackFunction}
            parameter="pH" 
            min = {0}
            max = {14}
            step = {0.1}
          />
          <ParameterSlider
            parentCallback={conductivityCallbackFunction} 
            parameter="Conductivity"
            min = {0}
            max = {2000}
            step = {20}
          />
          <ParameterSlider
            parentCallback = {periodCallbackFunction} 
            parameter="Pump Period"
            min = {0}
            max = {60}
            step = {1}
          />
          <ParameterSlider 
            parentCallback = {dutyCycleCallbackFunction}
            parameter="Duty Cycle" 
            min = {0}
            max = {100}
            step = {5}   
          />     
        </div>
        <div className={styles.buttoncontainedTextParent}>
          <Button 
            sx={{ width: 327 }} 
            variant="contained" 
            color="primary"
            onClick={handleGetReadingsButtonClick}
          >
            GET READINGS
          </Button>
          <Button 
            sx={{ width: 327 }} 
            variant="contained" 
            color="primary" 
            onClick={handleUpdateSettingsButtonClick}
          >
            UPDATE SETTINGS
          </Button>
          <Button
            className={styles.buttoncontainedText}
            sx={{ width: 324 }}
            variant="contained"
            color="primary"
            onClick={openFramePopup}
          >
            CURRENT SNAPSHOT
          </Button>
        </div>
      </div>
      {isFramePopupOpen && (
        <PortalPopup
          overlayColor="rgba(113, 113, 113, 0.3)"
          placement="Centered"
          onOutsideClick={closeFramePopup}
        >
          <FrameComponent 
            onClose={closeFramePopup}
            pH = {currentPh}
            conductivity = {currentConductivity}
            temperature = {currentTemperature}
            humidity = {currentHumidity}
            lightIntensity = {currentLightIntensity}
            oxygen = {currentOxygenConc}
            co2 = {currentCO2Conc}


          />
        </PortalPopup>
      )}
    </>
  );
};

export default StatusCard;
