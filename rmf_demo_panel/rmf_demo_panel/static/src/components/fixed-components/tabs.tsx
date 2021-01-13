import * as React from 'react';
import { makeStyles, withStyles, Theme, createStyles } from '@material-ui/core/styles';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import { WorldContextType, World } from './app-context';
import { getConfigFile } from '../services';

interface StyledTabsProps {
  value: number;
  onChange: (event: React.ChangeEvent<{}>, newValue: number) => void;
}

const StyledTabs = withStyles({
  indicator: {
    display: 'flex',
    justifyContent: 'center',
    backgroundColor: 'transparent',
    '& > span': {
      maxWidth: 40,
      width: '100%',
      backgroundColor: '#635ee7',
    },
  },
})((props: StyledTabsProps) => <Tabs {...props} TabIndicatorProps={{ children: <span /> }} />);

interface StyledTabProps {
  label: string;
}

const StyledTab = withStyles((theme: Theme) =>
  createStyles({
    root: {
      textTransform: 'none',
      color: '#2e1534',
      fontWeight: theme.typography.fontWeightRegular,
      fontSize: theme.typography.pxToRem(15),
      marginRight: theme.spacing(0.5),
      '&:focus': {
        opacity: 1,
      },
    },
  }),
)((props: StyledTabProps) => <Tab disableRipple {...props} />);

const useStyles = makeStyles((theme: Theme) => ({
  root: {
    flexGrow: 1,
  },
  padding: {
    padding: theme.spacing(2),
  },
  tabs: {
    backgroundColor: '#fafafa',
  },
  tabContainer: {
      paddingTop: theme.spacing(1),
      paddingBottom: theme.spacing(1)
  }
})); 

interface NavTabsProps {
  handleWorldChange: React.Dispatch<React.SetStateAction<WorldContextType>>
}

const NavTabs: React.FC<NavTabsProps> = (props: NavTabsProps) => {
  const { handleWorldChange } = props;
  const classes = useStyles();
  const [value, setValue] = React.useState(0);

  const handleChange = async (event: React.ChangeEvent<{}>, newValue: number) => {
    setValue(newValue);
    const config = await getConfigFile(World[newValue]);
    handleWorldChange({ map: newValue, config: config });
  };

  return (
    <div className={classes.root} role="nav-tabs">
      <div className={classes.tabs}>
        <StyledTabs value={value} onChange={handleChange} aria-label="styled tabs example">
          <StyledTab label="Office" />
          <StyledTab label="Airport" />
          <StyledTab label="Clinic" />
        </StyledTabs>
      </div>
    </div>
  );
}

export default NavTabs; 
