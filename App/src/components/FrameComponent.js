import styles from "./FrameComponent.module.css";

const FrameComponent = ({ onClose,pH,conductivity,temperature,humidity,lightIntensity,oxygen,co2 }) => {
  return (
    <div className={styles.parameterSnapshotParent}>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>pH</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{pH}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>Conductivity</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{conductivity}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>Temperature</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{temperature}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>Light Intensity</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{lightIntensity}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>Humidity</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{humidity}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>Oxygen Concentration</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{oxygen}</b>
        </div>
      </div>
      <div className={styles.parameterSnapshot}>
        <div className={styles.phWrapper}>
          <b className={styles.ph}>CO2 Concentration</b>
        </div>
        <div className={styles.wrapper}>
          <b className={styles.ph}>{co2}</b>
        </div>
      </div>
    </div>
  );
};


export default FrameComponent;
