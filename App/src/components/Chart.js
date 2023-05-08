// import { Autocomplete, TextField } from "@mui/material";

// const axios = require('axios');
// const pandas = require('pandas-js');

// const url = 'https://7day35xrbospq4slpqt2abr7fq0mzeja.lambda-url.us-east-1.on.aws/';

// axios.get(url)
//   .then(response => {
//     const data = response.data;
//     const df = pandas.DataFrame.fromRecords(data);
//     console.log(df.head());
//   })
//   .catch(error => {
//     console.error(error);
//   });

// const handleMeasureValueChange = e => {
//   setMeasureValue(e.target.value);
// };



// const Chart = () => {

//   const [measureValue, setMeasureValue] = useState('');


//   return (
//     <div>
//       <Autocomplete
//         className={styles.autocompletestandard}
//         disablePortal
//         options={[
//           "pH",
//           "Conductivity",
//           "Light Intensity",
//           "Temperature",
//           "Humidity",
//           "Oxygen",
//           "CO2",
//         ]}
//         renderInput={(params) => (
//           <TextField
//             {...params}
//             color="primary"
//             label="Label"
//             variant="standard"
//             placeholder=""
//             helperText=""
//           />
//         )}
//         defaultValue="pH"
//         size="medium"
//       />
//     </div>
//   );
// };

// export default Chart;


// import React, { useState, useEffect } from 'react';
// import axios from 'axios';
// import { Line } from 'react-chartjs-2';
// import { Autocomplete, TextField } from "@mui/material";
// import styles from "./Chart.module.css";
// import defaultData from "./defaultData.js";

// function Chart() {
//   const [data, setData] = useState([defaultData]);
//   const [dependentVar, setDependentVar] = useState('temperature');
//   const [filteredData, setFilteredData] = useState([]);

//   useEffect(() => {
//     axios.get('https://7day35xrbospq4slpqt2abr7fq0mzeja.lambda-url.us-east-1.on.aws/', {
//     })
//       .then((response) => {
        
//         const newData = response.data.map((d) => {
//           return {
//             name: d.Name,
//             measure_name: d.measure_name,
//             time: new Date(d.time),
//             measure_value: parseInt(d['measure_value::bigint'])
//           };
//         });
//         console.log(newData);
//         setData(newData);
//       })
//       .catch((error) => {
//         console.log(error);
//       });
//   }, []);

//   useEffect(() => {
//     setFilteredData(data.filter((d) => d.measure_name === dependentVar));
//   }, [dependentVar, data]);

//   const chartData = {
//     labels: filteredData.map((d) => d.time.toLocaleString()),
//     datasets: [
//       {
//         label: dependentVar,
//         data: filteredData.map((d) => d.measure_value),
//         fill: false,
//         borderColor: 'rgba(75,192,192,1)',
//         borderWidth: 2,
//       },
//     ],
//   };

//   const options = {
//     scales: {
//       xAxes: [
//         {
//           type: 'time',
//           distribution: 'linear',
//           time: {
//             displayFormats: {
//               minute: 'h:mm a',
//               hour: 'hA',
//               day: 'MMM D',
//             },
//           },
//         },
//       ],
//     },
//   };

//   return (
//     <div>
//       <Autocomplete
//         id="dependent-var"
//         className={styles.autocompletestandard}
//         disablePortal
//         options={Array.from(new Set(data.map((d) => d.measure_name)))}
//         defaultValue="temperature"
//         onChange={(event, newValue) => {
//           // setDependentVar(newValue);
//           setFilteredData(data.filter((d) => d.measure_name === dependentVar));
//         }}
//         renderInput={(params) => (
//           <TextField
//             {...params}
//             color="primary"
//             label="Parameter"
//             variant="standard"
//             placeholder=""
//             helperText=""
//           />
//         )}
//       />
//       <Line data={chartData} options={options} />
//     </div>
//   );
// }

// export default Chart;

// import React, { useEffect, useState } from 'react';
// import axios from 'axios';
// import { Autocomplete, TextField } from "@mui/material";
// import * as d3 from 'd3';
// import Plot from 'react-plotly.js';

// function unpack(rows, key) {
//   return rows.map(function (row) {
//     return row[key];
//   });
// }

// const Chart = () => {

//   const [data, setData] = useState([]);
//   const [plotData, setPlotData] = useState([]);
//   const [selectedY, setSelectedY] = useState('temperature');

 

//   // Fetch data from the API
//   useEffect(() => {
//     const fetchData = async () => {
//       try {
//         const response = await axios.get('https://7day35xrbospq4slpqt2abr7fq0mzeja.lambda-url.us-east-1.on.aws/');
//         const responseData = response.data;

//         // Store the received data in the state
//         setData(responseData);
//       } catch (error) {
//         console.log(error);
//       }
//     };

//     fetchData();
//   }, []);

//   useEffect(() => {
//     if (data.length === 0 || selectedY === '') {
//       return; // Wait for data and selectedY to be set
//     }
  
//     // Format the data for Plotly
//     const plotlyData = {
//       x: unpack(data, 'time'),
//       y: unpack(data, selectedY),
//       type: 'scatter',
//       mode: 'lines'
//     };

//     setPlotData(plotlyData);
//     console.log(plotData);
  
//   }, [data, selectedY]);
  

//   return (
//     <div>
//       <Autocomplete
//         options={['temperature', 'humidity']} // Available options for y variable
//         defaultValue='temperature'
//         onChange={(event, value) => setSelectedY(value)}
//         renderInput={(params) => <TextField {...params} label="Select Y Variable" />}
//       />
//       <Plot
//         data={plotData}
//         layout={{width: 320, height: 240}}
//         onInitialized={(figure) => this.setState(figure)}
//         onUpdate={(figure) => this.setState(figure)}
//       />
//     </div>
//   );
// };

// export default Chart;

import React, { useEffect, useState } from 'react';
import axios from 'axios';
import { Autocomplete, TextField } from "@mui/material";
import Plot from 'react-plotly.js';
import styles from "./Chart.module.css";

function unpack(rows, key) {
  // console.log(typeof rows);
  // console.log(key);
  return rows.map(function (row) {
    return row[key];
  });
}

const Chart = () => {
  const [data, setData] = useState([]);
  const [plotData, setPlotData] = useState([]);
  const [selectedY, setSelectedY] = useState('temperature');
  
  const config={responsive: true}

  // Fetch data from the API
  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await axios.get('https://7day35xrbospq4slpqt2abr7fq0mzeja.lambda-url.us-east-1.on.aws/', {
          headers: {
            Accept: 'application/json',
            'Content-Type': 'application/json',
          },
        });
  
        const responseData = response.data;
        setData(responseData);
      } catch (error) {
        if (error.response && error.response.status === 502) {
          // Retry request if 502 error is encountered
          try {
            const retryResponse = await axios.get('https://7day35xrbospq4slpqt2abr7fq0mzeja.lambda-url.us-east-1.on.aws/', {
              headers: {
                Accept: 'application/json',
                'Content-Type': 'application/json',
              },
            });
  
            const responseData = retryResponse.data;
            setData(responseData);
          } catch (retryError) {
            console.log(retryError);
          }
        } else {
          console.log(error);
        }
      }
    };
  
    fetchData();
  }, []);
  

  useEffect(() => {
    if (data.length === 0 || selectedY === '') {
      return; // Wait for data and selectedY to be set
    }

    // console.log(data)

    // Format the data for Plotly
    const plotlyData = {
      x: unpack(data, 'time'),
      y: unpack(data, selectedY),
      type: 'scatter',
      mode: 'lines'
    };

    // console.log([plotlyData]);

    setPlotData([plotlyData]);
  }, [data, selectedY]);

  return (
    // <div className={styles.lineCharttwovaluesParent}>
    <div>
      
      <Autocomplete
        className={styles.autocompletestandard}
        options={['temperature', 'humidity', 'co2', 'oxygen', 'ph', 'conductivity']} // Available options for y variable
        defaultValue='temperature'
        onChange={(event, value) => setSelectedY(value)}
        renderInput={(params) => <TextField {...params} label="Select Y Variable" />}
      />
      <Plot
        data={plotData}
        layout={{ width: 1000, height: 500 }}
        onInitialized={(figure) => this.setState(figure)}
        onUpdate={(figure) => this.setState(figure)}
      />
    </div>
  );
};

export default Chart;

