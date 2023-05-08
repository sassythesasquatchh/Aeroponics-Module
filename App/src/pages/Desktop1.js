import Chart from "../components/Chart";
import StatusCard from "../components/StatusCard";
import styles from "./Desktop1.module.css";

const Desktop1 = () => {
  return (
    <div className={styles.desktop1}>
      <div className={styles.sidePanel} />
      <div className={styles.modularAeroponicsParent}>
        <h1 className={styles.modularAeroponics}>Modular Aeroponics</h1>
        <h5 className={styles.anIotModular}>
          An IoT modular aeroponics agricultural system can revolutionize the
          way we grow crops by combining the power of the internet of things
          (IoT) with the efficiency of aeroponic farming. This system provides
          numerous benefits, including the ability to grow fresh produce in any
          environment, regardless of the climate or soil quality. The modular
          design of this system allows for easy scalability and customization,
          making it ideal for use in both small and large-scale farming
          operations. Additionally, the use of IoT technology enables farmers to
          remotely monitor and control the system, providing real-time insights
          into plant health, nutrient levels, and more.
        </h5>
        <Chart />
        <StatusCard />
      </div>
      <div className={styles.sidePanel1} />
    </div>
  );
};

export default Desktop1;
